# Neuromeka Clients
![PyPI](https://img.shields.io/pypi/v/neuromeka)

This package provides client protocols for users to interact with Neuromeka's products, including Indy, Moby, Ecat, and Motor.

## Installation

You can install the package from PyPI:

```bash
pip install neuromeka
```

## Usage
The package contatins the following client classes:

* IndyDCP3 in indydcp3.py
* IndyEye in eye.py
* EtherCAT in ecat.py
* Moby in moby.py

To use a client class, simply import it and create an instance:

```python
from neuromeka import IndyDCP3, IndyEye, EtherCAT, MobyClient

ecat = EtherCAT("192.168.214.20")
moby = MobyClient("192.168.214.20")
indy = IndyDCP3("192.168.0.11")
eye = IndyEye("192.168.0.12")
```

## Dependencies
This package requires the following dependencies:

* grpcio
* grpcio-tools
* protobuf
* requests
* Pillow
* numpy
* pyModbusTCP
* netifaces

These dependencies will be automatically installed when you install the package using pip.

## Examples
Please refer to the 'example.py' file in the package for usage examples.

## Support
If you encounter any issues or need help, please open an issue on the project's repository.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
