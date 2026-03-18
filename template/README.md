# Template Component

[![Component Registry](https://components.espressif.com/components/pedrominatel/template/badge.svg)](https://components.espressif.com/components/pedrominatel/template)

This component is a minimal starter template for ESP-IDF components.

## File Layout

- `include/template.h` public API and exported types
- `template.c` private implementation and lifecycle placeholders
- `CMakeLists.txt` component registration and dependencies
- `idf_component.yml` component metadata for the registry

## API

```c
#include "template.h"

template_handle_t handle = template_create();
if (handle == NULL) {
    // Allocation or initialization failed
}

esp_err_t ret = template_delete(handle);
```

### Exported symbols

- `template_create()` allocates and initializes the template handle
- `template_delete()` frees the handle and validates input

## Best Practices for Future Components

- Replace placeholder names, comments, and metadata before publishing
- Add only the dependencies your component actually requires
- Keep public headers free of unrelated driver includes
- Use opaque handles for private state
- Return `esp_err_t` from library code instead of aborting internally
- Add Kconfig options only when users need configurable behavior
- Document examples only when examples exist in the component

## Customization Checklist

- Rename the component directory, files, include guards, and exported symbols
- Replace `template_create()` and `template_delete()` with the real lifecycle API
- Add component-specific configuration structs, enums, and functions as needed
- Add required dependencies to `CMakeLists.txt`
- Update `idf_component.yml` description, version, and registry metadata
- Expand this README with real hardware, protocol, or usage details

## CI Publishing Workflow

This repository publishes components to the Espressif Component Registry through GitHub CI. The workflow runs on every push to `main` and uses the `espressif/upload-components-ci-action@v2` action.

Official documentation:
- [ESP-IDF Component Manager](https://docs.espressif.com/projects/idf-component-manager/en/latest/index.html)

### Current workflow behavior

- Trigger: push to `main`
- Namespace: set by the `namespace` field in `.github/workflows/upload_components.yml`
- Authentication: configured by the workflow inputs used with `espressif/upload-components-ci-action@v2`
- Published components: defined by the workflow inputs in `.github/workflows/upload_components.yml`

## How To Add A Component To The Registry

1. Make sure the component has a valid `idf_component.yml` with at least `version`, `description`, `url`, and `dependencies.idf`.
2. Place the component in its own top-level directory in this repository.
3. Add the component to the workflow inputs in [upload_components.yml](/Users/pedrominatel/Documents/Espressif/github/esp-components/.github/workflows/upload_components.yml) using the format required by `espressif/upload-components-ci-action@v2`.
4. Commit the component files and the workflow update to `main`.
5. Push to `main` so GitHub Actions uploads the component to the registry.

### Example

If your component is named `template`, update the workflow configuration so the action uploads that component from its repository path.

### Required repository setup

- Configure the workflow inputs required by `espressif/upload-components-ci-action@v2`
- Set the workflow namespace to the registry namespace you want to publish to
- Never hardcode `api_token` in the workflow YAML; store it in GitHub Secrets and reference it as `${{ secrets.YOUR_SECRET_NAME }}`
- Keep the component name and path aligned with the actual component folder and metadata

### Recommended release practice

- Bump `version` in `idf_component.yml` before publishing changes
- Treat breaking API changes as a major version bump
- Keep the README and registry metadata in sync before pushing to `main`
