using RfRxV02Host.Core.Abstractions;
using RfRxV02Host.Core.Services;
using System.Text;

namespace RfRxV02Host.Maui;

public partial class ParametersPage : ContentPage
{
    private readonly ITransportClient _transport;
    private readonly IReadOnlyList<LerpGroupDefinition> _lerpGroups;
    private bool _isUpdatingOpModeUi;
    private bool _isUpdatingDioMaskUi;
    private ushort _dioOnMaskValue;

    private static readonly string[] OpDirectionItems = ["發射機", "接收機"];
    private static readonly string[] OpControlItems = ["控制器", "過捲"];
    private static readonly string[] OpGearItems = ["步進馬達", "伺服馬達", "行車電腦", "比例閥", "步進馬達/行車電腦"];
    private static readonly string[] OpTxTypeItems = ["發射機-單軸", "發射機-四軸"];
    private static readonly string[] OpValveTypeItems = ["開關閥", "比例閥"];

    public ParametersPage(ITransportClient transport)
    {
        InitializeComponent();
        _transport = transport;
        _lerpGroups =
        [
            new(
                "VR",
                [DfParamId.VrRawPt1, DfParamId.VrRawPt2, DfParamId.VrRawPt3, DfParamId.VrRawPt4],
                [DfParamId.VrAnglePt1, DfParamId.VrAnglePt2, DfParamId.VrAnglePt3, DfParamId.VrAnglePt4]),
            new(
                "PWM",
                [DfParamId.PwmMapRawPt1, DfParamId.PwmMapRawPt2, DfParamId.PwmMapRawPt3, DfParamId.PwmMapRawPt4],
                [DfParamId.PwmMapCounterPt1, DfParamId.PwmMapCounterPt2, DfParamId.PwmMapCounterPt3, DfParamId.PwmMapCounterPt4]),
            new(
                "PWM_TURTLE",
                [DfParamId.PwmMap4xRawPt1, DfParamId.PwmMap4xRawPt2, DfParamId.PwmMap4xRawPt3, DfParamId.PwmMap4xRawPt4],
                [DfParamId.PwmMap4xCounterPt1, DfParamId.PwmMap4xCounterPt2, DfParamId.PwmMap4xCounterPt3, DfParamId.PwmMap4xCounterPt4]),
            new(
                "SERVO",
                [DfParamId.ServoMapRawPt1, DfParamId.ServoMapRawPt2, DfParamId.ServoMapRawPt3, DfParamId.ServoMapRawPt4],
                [DfParamId.ServoMapCounterPt1, DfParamId.ServoMapCounterPt2, DfParamId.ServoMapCounterPt3, DfParamId.ServoMapCounterPt4])
        ];

        List<DfParameterInfo> items = DfParameterCatalog.All.ToList();
        ParamPicker.ItemsSource = items;
        ParamPicker.SelectedIndexChanged += (_, _) => UpdateRangeLabel();
        if (items.Count > 0)
        {
            ParamPicker.SelectedIndex = 0;
        }

        InitializeOpModeEditor();
        UpdateDioOnMaskUi();
    }

    private DfParameterInfo? Selected => ParamPicker.SelectedItem as DfParameterInfo;

    private void UpdateRangeLabel()
    {
        if (Selected is null)
        {
            RangeLabel.Text = "Range: -";
            return;
        }

        RangeLabel.Text = $"Range: {Selected.Min} - {Selected.Max}  Default: {Selected.Default}";
    }

    private async void OnReadClicked(object? sender, EventArgs e)
    {
        if (Selected is null)
        {
            return;
        }

        try
        {
            uint value = await _transport.ReadParamAsync((ushort)Selected.Id);
            ValueEntry.Text = value.ToString();
            UpdateRangeLabel();
        }
        catch (Exception ex)
        {
            ResultEditor.Text = $"讀取失敗: {ex.Message}";
        }
    }

    private async void OnWriteClicked(object? sender, EventArgs e)
    {
        if (Selected is null)
        {
            return;
        }

        if (!uint.TryParse(ValueEntry.Text, out uint writeVal))
        {
            await DisplayAlert("寫入", "無效數值", "OK");
            return;
        }

        if (writeVal < Selected.Min || writeVal > Selected.Max)
        {
            await DisplayAlert("寫入", "數值超出範圍", "OK");
            return;
        }

        try
        {
            await _transport.WriteParamAsync((ushort)Selected.Id, writeVal);
            ResultEditor.Text = $"寫入成功: {Selected.Name}={writeVal}";
        }
        catch (Exception ex)
        {
            ResultEditor.Text = $"寫入失敗: {ex.Message}";
        }
    }

    private async void OnReadAllClicked(object? sender, EventArgs e)
    {
        var sb = new StringBuilder();
        try
        {
            foreach (DfParameterInfo info in DfParameterCatalog.All)
            {
                uint value = await _transport.ReadParamAsync((ushort)info.Id);
                sb.AppendLine($"{info}: {value}");
            }

            ResultEditor.Text = sb.ToString();
        }
        catch (Exception ex)
        {
            ResultEditor.Text = $"全部讀回失敗: {ex.Message}";
        }
    }

    private async void OnReadVrGroupClicked(object? sender, EventArgs e)
    {
        await ReadGroupAsync(_lerpGroups[0]);
    }

    private async void OnWriteVrGroupClicked(object? sender, EventArgs e)
    {
        await WriteGroupAsync(_lerpGroups[0]);
    }

    private async void OnReadPwmGroupClicked(object? sender, EventArgs e)
    {
        await ReadGroupAsync(_lerpGroups[1]);
    }

    private async void OnWritePwmGroupClicked(object? sender, EventArgs e)
    {
        await WriteGroupAsync(_lerpGroups[1]);
    }

    private async void OnReadServoGroupClicked(object? sender, EventArgs e)
    {
        await ReadGroupAsync(_lerpGroups[3]);
    }

    private async void OnWriteServoGroupClicked(object? sender, EventArgs e)
    {
        await WriteGroupAsync(_lerpGroups[3]);
    }

    private async void OnReadPwmTurtleGroupClicked(object? sender, EventArgs e)
    {
        await ReadGroupAsync(_lerpGroups[2]);
    }

    private async void OnWritePwmTurtleGroupClicked(object? sender, EventArgs e)
    {
        await WriteGroupAsync(_lerpGroups[2]);
    }

    private async void OnReadAllLerpGroupsClicked(object? sender, EventArgs e)
    {
        var sb = new StringBuilder();
        try
        {
            foreach (LerpGroupDefinition group in _lerpGroups)
            {
                sb.AppendLine($"[{group.Name}]");
                for (int i = 0; i < 4; i++)
                {
                    uint x = await _transport.ReadParamAsync((ushort)group.XIds[i]);
                    uint y = await _transport.ReadParamAsync((ushort)group.YIds[i]);
                    sb.AppendLine($"Pt{i + 1}: X={x}, Y={y}");
                }

                sb.AppendLine();
            }

            ResultEditor.Text = sb.ToString();
        }
        catch (Exception ex)
        {
            ResultEditor.Text = $"讀取所有參數失敗: {ex.Message}";
        }
    }

    private void InitializeOpModeEditor()
    {
        OpModeDirectionPicker.ItemsSource = OpDirectionItems.ToList();
        OpModeControlPicker.ItemsSource = OpControlItems.ToList();
        OpModeGearPicker.ItemsSource = OpGearItems.ToList();

        _isUpdatingOpModeUi = true;
        OpModeDirectionPicker.SelectedIndex = 1;
        OpModeControlPicker.SelectedIndex = 0;
        OpModeGearPicker.SelectedIndex = 0;
        UpdateOpModeTypeItems();
        OpModeTypePicker.SelectedIndex = 0;
        _isUpdatingOpModeUi = false;

        UpdateOpModePreview();
    }

    private void UpdateOpModeTypeItems()
    {
        bool isTx = OpModeDirectionPicker.SelectedIndex == 0;
        if (isTx)
        {
            OpModeTypeLabel.Text = "發射類型";
            OpModeTypePicker.ItemsSource = OpTxTypeItems.ToList();
        }
        else
        {
            OpModeTypeLabel.Text = "閥體類型";
            OpModeTypePicker.ItemsSource = OpValveTypeItems.ToList();
        }
    }

    private void OnOpModePickerChanged(object? sender, EventArgs e)
    {
        if (_isUpdatingOpModeUi)
        {
            return;
        }

        if (sender == OpModeDirectionPicker)
        {
            _isUpdatingOpModeUi = true;
            int previousTypeIndex = OpModeTypePicker.SelectedIndex;
            UpdateOpModeTypeItems();
            OpModeTypePicker.SelectedIndex = previousTypeIndex is >= 0 and <= 1 ? previousTypeIndex : 0;
            _isUpdatingOpModeUi = false;
        }

        UpdateOpModePreview();
    }

    private byte ComposeOpModeFromUi()
    {
        int direction = OpModeDirectionPicker.SelectedIndex == 1 ? 0x80 : 0x00;
        int control = OpModeControlPicker.SelectedIndex == 1 ? 0x40 : 0x00;
        int gear = Math.Clamp(OpModeGearPicker.SelectedIndex, 0, 7);
        int type = Math.Clamp(OpModeTypePicker.SelectedIndex, 0, 3) << 3;
        return (byte)(direction | control | type | gear);
    }

    private void UpdateOpModePreview()
    {
        byte value = ComposeOpModeFromUi();
        OpModeValueLabel.Text = $"{value} (0x{value:X2}, {Convert.ToString(value, 2).PadLeft(8, '0')})";
    }

    private void ApplyOpModeToUi(byte value)
    {
        _isUpdatingOpModeUi = true;

        OpModeDirectionPicker.SelectedIndex = (value & 0x80) != 0 ? 1 : 0;
        OpModeControlPicker.SelectedIndex = (value & 0x40) != 0 ? 1 : 0;
        OpModeGearPicker.SelectedIndex = Math.Clamp(value & 0x07, 0, OpGearItems.Length - 1);

        UpdateOpModeTypeItems();
        int typeIndex = (value >> 3) & 0x03;
        OpModeTypePicker.SelectedIndex = typeIndex is 0 or 1 ? typeIndex : 0;

        _isUpdatingOpModeUi = false;
        UpdateOpModePreview();
    }

    private async void OnReadOpModeClicked(object? sender, EventArgs e)
    {
        try
        {
            uint value = await _transport.ReadParamAsync((ushort)DfParamId.OpMode);
            byte opModeValue = (byte)(value & 0xFF);
            ApplyOpModeToUi(opModeValue);
            ValueEntry.Text = opModeValue.ToString();
            ResultEditor.Text = $"讀取運作模式成功: {opModeValue} (0x{opModeValue:X2})";
        }
        catch (Exception ex)
        {
            ResultEditor.Text = $"讀取運作模式失敗: {ex.Message}";
        }
    }

    private async void OnWriteOpModeClicked(object? sender, EventArgs e)
    {
        try
        {
            byte value = ComposeOpModeFromUi();
            await _transport.WriteParamAsync((ushort)DfParamId.OpMode, value);
            ValueEntry.Text = value.ToString();
            ResultEditor.Text = $"寫入運作模式成功: {value} (0x{value:X2})";
        }
        catch (Exception ex)
        {
            ResultEditor.Text = $"寫入運作模式失敗: {ex.Message}";
        }
    }

    private void OnUseOpModeInSingleToolClicked(object? sender, EventArgs e)
    {
        DfParameterInfo? opModeInfo = DfParameterCatalog.All.FirstOrDefault(x => x.Id == DfParamId.OpMode);
        if (opModeInfo is not null)
        {
            ParamPicker.SelectedItem = opModeInfo;
        }

        byte value = ComposeOpModeFromUi();
        ValueEntry.Text = value.ToString();
        UpdateRangeLabel();
    }

    private void UpdateDioOnMaskUi()
    {
        _isUpdatingDioMaskUi = true;
        Aux1CheckBox.IsChecked = (_dioOnMaskValue & (1 << 8)) != 0;
        Aux2CheckBox.IsChecked = (_dioOnMaskValue & (1 << 9)) != 0;
        _isUpdatingDioMaskUi = false;
        DioOnMaskValueLabel.Text = $"遮罩: {_dioOnMaskValue} (0x{_dioOnMaskValue:X4})";
    }

    private void OnAuxMaskCheckedChanged(object? sender, CheckedChangedEventArgs e)
    {
        if (_isUpdatingDioMaskUi)
        {
            return;
        }

        ushort mask = _dioOnMaskValue;
        if (Aux1CheckBox.IsChecked)
        {
            mask |= (ushort)(1 << 8);
        }
        else
        {
            mask = (ushort)(mask & ~(1 << 8));
        }

        if (Aux2CheckBox.IsChecked)
        {
            mask |= (ushort)(1 << 9);
        }
        else
        {
            mask = (ushort)(mask & ~(1 << 9));
        }

        _dioOnMaskValue = mask;
        DioOnMaskValueLabel.Text = $"遮罩: {_dioOnMaskValue} (0x{_dioOnMaskValue:X4})";
    }

    private async void OnReadDioOnMaskClicked(object? sender, EventArgs e)
    {
        try
        {
            uint value = await _transport.ReadParamAsync((ushort)DfParamId.DioOnMask);
            _dioOnMaskValue = (ushort)value;
            UpdateDioOnMaskUi();
            ResultEditor.Text = $"讀取同動控制成功: {_dioOnMaskValue} (0x{_dioOnMaskValue:X4})";
        }
        catch (Exception ex)
        {
            ResultEditor.Text = $"讀取同動控制失敗: {ex.Message}";
        }
    }

    private async void OnWriteDioOnMaskClicked(object? sender, EventArgs e)
    {
        try
        {
            await _transport.WriteParamAsync((ushort)DfParamId.DioOnMask, _dioOnMaskValue);
            ResultEditor.Text = $"寫入同動控制成功: {_dioOnMaskValue} (0x{_dioOnMaskValue:X4})";
        }
        catch (Exception ex)
        {
            ResultEditor.Text = $"寫入同動控制失敗: {ex.Message}";
        }
    }

    private async void OnReadLbtBoundsClicked(object? sender, EventArgs e)
    {
        try
        {
            uint low = await _transport.ReadParamAsync((ushort)DfParamId.TxLbtLowBound);
            uint high = await _transport.ReadParamAsync((ushort)DfParamId.TxLbtHighBound);
            LbtLowBoundEntry.Text = low.ToString();
            LbtHighBoundEntry.Text = high.ToString();
            ResultEditor.Text = $"讀取低壓設定成功: 下限={low}mV, 上限={high}mV";
        }
        catch (Exception ex)
        {
            ResultEditor.Text = $"讀取低壓設定失敗: {ex.Message}";
        }
    }

    private async void OnWriteLbtBoundsClicked(object? sender, EventArgs e)
    {
        if (!ushort.TryParse(LbtLowBoundEntry.Text, out ushort low))
        {
            await DisplayAlert("寫入低壓設定", "電池低壓檢測-下限必須為 0..65535 mV", "OK");
            return;
        }

        if (!ushort.TryParse(LbtHighBoundEntry.Text, out ushort high))
        {
            await DisplayAlert("寫入低壓設定", "電池低壓檢測-上限必須為 0..65535 mV", "OK");
            return;
        }

        if (low > high)
        {
            await DisplayAlert("寫入低壓設定", "電池低壓檢測-下限必須小於或等於上限", "OK");
            return;
        }

        try
        {
            await _transport.WriteParamAsync((ushort)DfParamId.TxLbtLowBound, low);
            await _transport.WriteParamAsync((ushort)DfParamId.TxLbtHighBound, high);
            ResultEditor.Text = $"寫入低壓設定成功: 下限={low}mV, 上限={high}mV";
        }
        catch (Exception ex)
        {
            ResultEditor.Text = $"寫入低壓設定失敗: {ex.Message}";
        }
    }

    private async Task ReadGroupAsync(LerpGroupDefinition group)
    {
        try
        {
            Entry[] xEntries = GetXEntries(group.Name);
            Entry[] yEntries = GetYEntries(group.Name);
            for (int i = 0; i < 4; i++)
            {
                uint x = await _transport.ReadParamAsync((ushort)group.XIds[i]);
                uint y = await _transport.ReadParamAsync((ushort)group.YIds[i]);
                xEntries[i].Text = x.ToString();
                yEntries[i].Text = y.ToString();
            }

            ResultEditor.Text = $"讀取群組成功: {group.Name}";
        }
        catch (Exception ex)
        {
            ResultEditor.Text = $"讀取群組失敗: {ex.Message}";
        }
    }

    private async Task WriteGroupAsync(LerpGroupDefinition group)
    {
        try
        {
            (ushort[] xs, ushort[] ys) = ParseLerpPoints(group.Name);
            for (int i = 1; i < xs.Length; i++)
            {
                if (xs[i] < xs[i - 1])
                {
                    await DisplayAlert("寫入群組", $"{group.Name}: X (RAW) 必須由 Pt1 到 Pt4 非遞減", "OK");
                    return;
                }
            }

            for (int i = 0; i < 4; i++)
            {
                await _transport.WriteParamAsync((ushort)group.XIds[i], xs[i]);
                await _transport.WriteParamAsync((ushort)group.YIds[i], ys[i]);
            }

            ResultEditor.Text = $"寫入群組成功: {group.Name}";
        }
        catch (FormatException ex)
        {
            await DisplayAlert("寫入群組", ex.Message, "OK");
        }
        catch (Exception ex)
        {
            ResultEditor.Text = $"寫入群組失敗: {ex.Message}";
        }
    }

    private static ushort ParseU16(Entry entry, string name)
    {
        if (!ushort.TryParse(entry.Text, out ushort value))
        {
            throw new FormatException($"{name} must be 0..65535.");
        }

        return value;
    }

    private (ushort[] xs, ushort[] ys) ParseLerpPoints(string groupName)
    {
        Entry[] xEntries = GetXEntries(groupName);
        Entry[] yEntries = GetYEntries(groupName);
        ushort[] xs = new ushort[4];
        ushort[] ys = new ushort[4];

        for (int i = 0; i < 4; i++)
        {
            xs[i] = ParseU16(xEntries[i], $"{groupName} Pt{i + 1} X");
            ys[i] = ParseU16(yEntries[i], $"{groupName} Pt{i + 1} Y");
        }

        return (xs, ys);
    }

    private Entry[] GetXEntries(string groupName) => groupName switch
    {
        "VR" => [VrPt1XEntry, VrPt2XEntry, VrPt3XEntry, VrPt4XEntry],
        "PWM" => [PwmPt1XEntry, PwmPt2XEntry, PwmPt3XEntry, PwmPt4XEntry],
        "PWM_TURTLE" => [PwmTurtlePt1XEntry, PwmTurtlePt2XEntry, PwmTurtlePt3XEntry, PwmTurtlePt4XEntry],
        "SERVO" => [ServoPt1XEntry, ServoPt2XEntry, ServoPt3XEntry, ServoPt4XEntry],
        _ => throw new ArgumentOutOfRangeException(nameof(groupName), groupName, "Unknown interpolation group.")
    };

    private Entry[] GetYEntries(string groupName) => groupName switch
    {
        "VR" => [VrPt1YEntry, VrPt2YEntry, VrPt3YEntry, VrPt4YEntry],
        "PWM" => [PwmPt1YEntry, PwmPt2YEntry, PwmPt3YEntry, PwmPt4YEntry],
        "PWM_TURTLE" => [PwmTurtlePt1YEntry, PwmTurtlePt2YEntry, PwmTurtlePt3YEntry, PwmTurtlePt4YEntry],
        "SERVO" => [ServoPt1YEntry, ServoPt2YEntry, ServoPt3YEntry, ServoPt4YEntry],
        _ => throw new ArgumentOutOfRangeException(nameof(groupName), groupName, "Unknown interpolation group.")
    };

    private sealed record LerpGroupDefinition(string Name, DfParamId[] XIds, DfParamId[] YIds);
}
