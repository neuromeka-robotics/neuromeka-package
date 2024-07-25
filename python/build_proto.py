import os
from grpc_tools import protoc

def compile_protos(proto_dir, output_dir):
    protoc.main((
        '',
        f'--proto_path={proto_dir}',
        f'--python_out={output_dir}',
        f'--grpc_python_out={output_dir}',
        f'{proto_dir}/*.proto'
    ))

if __name__ == '__main__':
    
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # directory containing .proto files
    proto_directory = os.path.join(current_dir, '..', 'proto')
    
    # output directory
    output_directory = os.path.join(proto_directory, 'python_generated')

    # Create the output directory
    os.makedirs(output_directory, exist_ok=True)

    # Compile .proto files
    compile_protos(proto_directory, output_directory)
