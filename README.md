# juno
Juno provides a dataflow-based autonomous driving framework for computations that span from the cloud to the device, which is developed based on [pylot](https://github.com/erdos-project/pylot) and [zenoh-flow-python](https://github.com/ZettaScaleLabs/zenoh-flow-python).

Pylot is an autonomous vehicle platform developed based on [ERDOS](https://github.com/erdos-project/erdos) dataflow framework. We use zenoh-flow-python instead of ERDOS to make Pylot support distributed computing.

## Getting Started
To make this project work, you will need to:
- install [pylot](https://github.com/erdos-project/pylot) using docker.
- install [zenoh-flow-python](https://github.com/ZettaScaleLabs/zenoh-flow-python) in pylot docker.
- Run the following command to run with Zenoh-flow in pylot docker:
```bash
../path/to/runtime -r foo -g pylot-pipeline.yml -l loader-config.yml
```
