namespace RfRxV02Host.Core.Services;

public sealed class DfParameterInfo
{
    public required DfParamId Id { get; init; }
    public required string Name { get; init; }
    public required uint Min { get; init; }
    public required uint Max { get; init; }
    public required uint Default { get; init; }

    public string PickerDisplayName => $"{(ushort)Id:D4} {Name}";

    public override string ToString() => $"{(ushort)Id:D4} {Name}";
}
