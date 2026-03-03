import sys
if sys.version_info >= (3, 9):
    from neuromeka.proto import *
else:
    from neuromeka.proto_step import *


class HelpersChannelAPI:
    """ChannelAPI providing internal signal conversion helpers."""

    def __to_digital_request_list__(self, digital_signal_list) -> list:
        req = []
        if not digital_signal_list:
            return req
        for sig in digital_signal_list:
            if isinstance(sig, device_msgs.DigitalSignal):
                req.append(sig)
            elif isinstance(sig, dict):
                req.append(device_msgs.DigitalSignal(
                    address=sig['address'],
                    state=sig['state'],
                    tool_index=sig.get('tool_index', 0)
                ))
            elif isinstance(sig, (tuple, list)):
                if len(sig) == 2:
                    address, state = sig
                    tool_index = 0
                elif len(sig) == 3:
                    address, state, tool_index = sig
                else:
                    raise ValueError("Digital tuple need 2 or 3 elements (address, state[, tool_index])")
                req.append(device_msgs.DigitalSignal(address=address, state=state, tool_index=tool_index))
            else:
                raise TypeError(f"Digital signal type error: {type(sig)}")
        return req

    def __to_analog_request_list__(self, analog_signal_list) -> list:
        req = []
        if not analog_signal_list:
            return req
        for sig in analog_signal_list:
            if isinstance(sig, device_msgs.AnalogSignal):
                req.append(sig)
            elif isinstance(sig, dict):
                req.append(device_msgs.AnalogSignal(
                    address=sig['address'],
                    voltage=sig['voltage'],
                    tool_index=sig.get('tool_index', 0)
                ))
            elif isinstance(sig, (tuple, list)):
                if len(sig) == 2:
                    address, voltage = sig
                    tool_index = 0
                elif len(sig) == 3:
                    address, voltage, tool_index = sig
                else:
                    raise ValueError("Analog tuple need 2 or 3 elements (address, voltage[, tool_index])")
                req.append(device_msgs.AnalogSignal(address=address, voltage=voltage, tool_index=tool_index))
            else:
                raise TypeError(f"Analog signal type error: {type(sig)}")
        return req

    def __to_endtool_signal_list__(self, endtool_signal_list) -> list:
        req = []
        if not endtool_signal_list:
            return req
        for sig in endtool_signal_list:
            if isinstance(sig, device_msgs.EndtoolSignal):
                req.append(sig)
            elif isinstance(sig, dict):
                req.append(device_msgs.EndtoolSignal(
                    port=str(sig['port']),
                    states=list(sig['states']),
                    tool_index=sig.get('tool_index', 0)
                ))
            elif isinstance(sig, (tuple, list)):
                if len(sig) == 2:
                    port, states = sig
                    tool_index = 0
                elif len(sig) == 3:
                    port, states, tool_index = sig
                else:
                    raise ValueError("Endtool tuple need 2 or 3 elements (port, states[, tool_index])")
                req.append(device_msgs.EndtoolSignal(port=str(port), states=list(states), tool_index=tool_index))
            else:
                raise TypeError(f"Endtool signal type error: {type(sig)}")
        return req
