# MAX17043 Alert Example

This example demonstrates how to configure and monitor battery low alert threshold.

## Description

This example will:
1. Initialize the MAX17043 fuel gauge
2. Set an alert threshold at 10% battery capacity
3. Continuously monitor the battery SOC and alert status
4. Log warnings when the battery level drops below the threshold
5. Automatically clear the alert after detection

The alert threshold can be modified in the code (range 0-32%).
