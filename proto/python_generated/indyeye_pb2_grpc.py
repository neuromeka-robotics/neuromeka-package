# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
"""Client and server classes corresponding to protobuf-defined services."""
import grpc

import indyeye_pb2 as indyeye__pb2


class EyeTaskStub(object):
    """Missing associated documentation comment in .proto file."""

    def __init__(self, channel):
        """Constructor.

        Args:
            channel: A grpc.Channel.
        """
        self.GetServerInfo = channel.unary_unary(
                '/IndyFramework.Protobuf.EyeTask.EyeTask/GetServerInfo',
                request_serializer=indyeye__pb2.ServerInfoRequest.SerializeToString,
                response_deserializer=indyeye__pb2.ServerInfoResponse.FromString,
                )
        self.GetImage = channel.unary_unary(
                '/IndyFramework.Protobuf.EyeTask.EyeTask/GetImage',
                request_serializer=indyeye__pb2.ImageRequest.SerializeToString,
                response_deserializer=indyeye__pb2.ImageResponse.FromString,
                )
        self.GetClassList = channel.unary_unary(
                '/IndyFramework.Protobuf.EyeTask.EyeTask/GetClassList',
                request_serializer=indyeye__pb2.Request.SerializeToString,
                response_deserializer=indyeye__pb2.ClassList.FromString,
                )
        self.Detect = channel.unary_unary(
                '/IndyFramework.Protobuf.EyeTask.EyeTask/Detect',
                request_serializer=indyeye__pb2.DetectRequest.SerializeToString,
                response_deserializer=indyeye__pb2.DetectResponse.FromString,
                )
        self.Retrieve = channel.unary_unary(
                '/IndyFramework.Protobuf.EyeTask.EyeTask/Retrieve',
                request_serializer=indyeye__pb2.RetrieveRequest.SerializeToString,
                response_deserializer=indyeye__pb2.DetectResponse.FromString,
                )
        self.GetWeldingLinesInfo = channel.unary_unary(
                '/IndyFramework.Protobuf.EyeTask.EyeTask/GetWeldingLinesInfo',
                request_serializer=indyeye__pb2.WeldingLinesInfoRequest.SerializeToString,
                response_deserializer=indyeye__pb2.WeldingLinesInfoResponse.FromString,
                )
        self.GetStraightLineInfo = channel.unary_unary(
                '/IndyFramework.Protobuf.EyeTask.EyeTask/GetStraightLineInfo',
                request_serializer=indyeye__pb2.StraightLineInfoRequest.SerializeToString,
                response_deserializer=indyeye__pb2.StraightLineInfoResponse.FromString,
                )
        self.GetCircularLineInfo = channel.unary_unary(
                '/IndyFramework.Protobuf.EyeTask.EyeTask/GetCircularLineInfo',
                request_serializer=indyeye__pb2.CircularLineInfoRequest.SerializeToString,
                response_deserializer=indyeye__pb2.CircularLineInfoResponse.FromString,
                )
        self.DoCalibration = channel.unary_unary(
                '/IndyFramework.Protobuf.EyeTask.EyeTask/DoCalibration',
                request_serializer=indyeye__pb2.CalibrationRequest.SerializeToString,
                response_deserializer=indyeye__pb2.CalibrationResponse.FromString,
                )
        self.IsCalibrationDone = channel.unary_unary(
                '/IndyFramework.Protobuf.EyeTask.EyeTask/IsCalibrationDone',
                request_serializer=indyeye__pb2.CalibrationDoneRequest.SerializeToString,
                response_deserializer=indyeye__pb2.CalibrationDoneResponse.FromString,
                )


class EyeTaskServicer(object):
    """Missing associated documentation comment in .proto file."""

    def GetServerInfo(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def GetImage(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def GetClassList(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def Detect(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def Retrieve(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def GetWeldingLinesInfo(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def GetStraightLineInfo(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def GetCircularLineInfo(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def DoCalibration(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def IsCalibrationDone(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')


def add_EyeTaskServicer_to_server(servicer, server):
    rpc_method_handlers = {
            'GetServerInfo': grpc.unary_unary_rpc_method_handler(
                    servicer.GetServerInfo,
                    request_deserializer=indyeye__pb2.ServerInfoRequest.FromString,
                    response_serializer=indyeye__pb2.ServerInfoResponse.SerializeToString,
            ),
            'GetImage': grpc.unary_unary_rpc_method_handler(
                    servicer.GetImage,
                    request_deserializer=indyeye__pb2.ImageRequest.FromString,
                    response_serializer=indyeye__pb2.ImageResponse.SerializeToString,
            ),
            'GetClassList': grpc.unary_unary_rpc_method_handler(
                    servicer.GetClassList,
                    request_deserializer=indyeye__pb2.Request.FromString,
                    response_serializer=indyeye__pb2.ClassList.SerializeToString,
            ),
            'Detect': grpc.unary_unary_rpc_method_handler(
                    servicer.Detect,
                    request_deserializer=indyeye__pb2.DetectRequest.FromString,
                    response_serializer=indyeye__pb2.DetectResponse.SerializeToString,
            ),
            'Retrieve': grpc.unary_unary_rpc_method_handler(
                    servicer.Retrieve,
                    request_deserializer=indyeye__pb2.RetrieveRequest.FromString,
                    response_serializer=indyeye__pb2.DetectResponse.SerializeToString,
            ),
            'GetWeldingLinesInfo': grpc.unary_unary_rpc_method_handler(
                    servicer.GetWeldingLinesInfo,
                    request_deserializer=indyeye__pb2.WeldingLinesInfoRequest.FromString,
                    response_serializer=indyeye__pb2.WeldingLinesInfoResponse.SerializeToString,
            ),
            'GetStraightLineInfo': grpc.unary_unary_rpc_method_handler(
                    servicer.GetStraightLineInfo,
                    request_deserializer=indyeye__pb2.StraightLineInfoRequest.FromString,
                    response_serializer=indyeye__pb2.StraightLineInfoResponse.SerializeToString,
            ),
            'GetCircularLineInfo': grpc.unary_unary_rpc_method_handler(
                    servicer.GetCircularLineInfo,
                    request_deserializer=indyeye__pb2.CircularLineInfoRequest.FromString,
                    response_serializer=indyeye__pb2.CircularLineInfoResponse.SerializeToString,
            ),
            'DoCalibration': grpc.unary_unary_rpc_method_handler(
                    servicer.DoCalibration,
                    request_deserializer=indyeye__pb2.CalibrationRequest.FromString,
                    response_serializer=indyeye__pb2.CalibrationResponse.SerializeToString,
            ),
            'IsCalibrationDone': grpc.unary_unary_rpc_method_handler(
                    servicer.IsCalibrationDone,
                    request_deserializer=indyeye__pb2.CalibrationDoneRequest.FromString,
                    response_serializer=indyeye__pb2.CalibrationDoneResponse.SerializeToString,
            ),
    }
    generic_handler = grpc.method_handlers_generic_handler(
            'IndyFramework.Protobuf.EyeTask.EyeTask', rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))


 # This class is part of an EXPERIMENTAL API.
class EyeTask(object):
    """Missing associated documentation comment in .proto file."""

    @staticmethod
    def GetServerInfo(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/IndyFramework.Protobuf.EyeTask.EyeTask/GetServerInfo',
            indyeye__pb2.ServerInfoRequest.SerializeToString,
            indyeye__pb2.ServerInfoResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def GetImage(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/IndyFramework.Protobuf.EyeTask.EyeTask/GetImage',
            indyeye__pb2.ImageRequest.SerializeToString,
            indyeye__pb2.ImageResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def GetClassList(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/IndyFramework.Protobuf.EyeTask.EyeTask/GetClassList',
            indyeye__pb2.Request.SerializeToString,
            indyeye__pb2.ClassList.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def Detect(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/IndyFramework.Protobuf.EyeTask.EyeTask/Detect',
            indyeye__pb2.DetectRequest.SerializeToString,
            indyeye__pb2.DetectResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def Retrieve(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/IndyFramework.Protobuf.EyeTask.EyeTask/Retrieve',
            indyeye__pb2.RetrieveRequest.SerializeToString,
            indyeye__pb2.DetectResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def GetWeldingLinesInfo(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/IndyFramework.Protobuf.EyeTask.EyeTask/GetWeldingLinesInfo',
            indyeye__pb2.WeldingLinesInfoRequest.SerializeToString,
            indyeye__pb2.WeldingLinesInfoResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def GetStraightLineInfo(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/IndyFramework.Protobuf.EyeTask.EyeTask/GetStraightLineInfo',
            indyeye__pb2.StraightLineInfoRequest.SerializeToString,
            indyeye__pb2.StraightLineInfoResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def GetCircularLineInfo(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/IndyFramework.Protobuf.EyeTask.EyeTask/GetCircularLineInfo',
            indyeye__pb2.CircularLineInfoRequest.SerializeToString,
            indyeye__pb2.CircularLineInfoResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def DoCalibration(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/IndyFramework.Protobuf.EyeTask.EyeTask/DoCalibration',
            indyeye__pb2.CalibrationRequest.SerializeToString,
            indyeye__pb2.CalibrationResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def IsCalibrationDone(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/IndyFramework.Protobuf.EyeTask.EyeTask/IsCalibrationDone',
            indyeye__pb2.CalibrationDoneRequest.SerializeToString,
            indyeye__pb2.CalibrationDoneResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)
