# juno
Juno provides a dataflow-based autonomous driving framework for computations that span from the cloud to the device.
## Getting Started
To make this project work, you will need to:
- install [pylot](https://github.com/erdos-project/pylot) using docker.
- install [zenoh-flow-python](https://github.com/ZettaScaleLabs/zenoh-flow-python) in pylot docker.
- Run the following command to run with Zenoh-flow in pylot docker:
```bash
../path/to/runtime -r foo -g pylot-pipeline.yml -l loader-config.yml
```
