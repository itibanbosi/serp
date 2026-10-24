// Compile-only API check. No motor command is executed.
function serpApiCompileCheck(): void {
    let stop: () => void = serp.car_stop
}
