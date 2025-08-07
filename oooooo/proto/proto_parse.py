import os
import re
from collections import defaultdict

def parse_imports(proto_file):
    imports = []
    with open(proto_file, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('import "'):
                import_file = line.split('"')[1]
                if import_file.endswith(".proto"):
                    import_file = import_file.split('/')[-1]
                    imports.append(import_file)
                     
    return imports

def topological_sort(dependency_graph):
    visited = set()
    stack = []
    temp_mark = set()

    def visit(node):
        if node in temp_mark:
            raise ValueError("Cyclic dependency detected")
        if node not in visited:
            temp_mark.add(node)
            for neighbor in dependency_graph[node]:
                visit(neighbor)
            temp_mark.remove(node)
            visited.add(node)
            stack.append(node)

    nodes = list(dependency_graph.keys())
    for node in nodes:
        if node not in visited:
            visit(node)

    return stack[::-1]

def generate_build_file(proto_files, dependency_graph, output_file):
    sorted_protos = topological_sort(dependency_graph)
    with open(output_file, 'w') as f:
        f.write('load("@rules_proto//proto:defs.bzl", "proto_library")\n')
        f.write('load("@rules_cc//cc:defs.bzl", "cc_proto_library")\n\n')
        f.write('package(default_visibility = ["//visibility:public"])\n\n')

        for proto in sorted_protos:
            proto_name = proto[:-6]  # Remove the .proto extension
            deps = dependency_graph[proto]

            if(proto_name == 'timestamp'):
                continue
            
            deps_str = ', '.join([f'":{dep[:-6]}_proto"' for dep in deps])
            f.write(f'proto_library(\n')
            f.write(f'    name = "{proto_name}_proto",\n')
            f.write(f'    srcs = ["{proto}"],\n')
            f.write(f'    deps = [{deps_str}],\n')
            f.write(f')\n\n')
            # f.write(f'cc_proto_library(\n')
            # f.write(f'    name = "{proto_name}_cc_proto",\n')
            # f.write(f'    deps = [":{proto_name}_proto"],\n')
            # f.write(f')\n\n')
        # 定义一个汇总的 proto_library
        f.write(f'proto_library(\n')
        f.write(f'    name = "all_proto",\n')
        f.write(f'    deps = [')
        for proto in sorted_protos:
            proto_name = proto[:-6]
            if(proto_name == 'timestamp'):
                continue
            f.write(f'":{proto_name}_proto", ')
        f.write(f'],\n')
        f.write(f')\n\n')
        f.write(f'cc_proto_library(\n')
        f.write(f'    name = "all_cc_proto",\n')
        f.write(f'    deps = [')
        f.write(f'":all_proto", ')
        f.write(f'],\n')
        f.write(f')\n\n')
        

def main(proto_dir, output_file):
    proto_files = [f for f in os.listdir(proto_dir) if f.endswith('.proto')]
    dependency_graph = defaultdict(list)

    for proto in proto_files:
        proto_path = os.path.join(proto_dir, proto)
        imports = parse_imports(proto_path)
        dependency_graph[proto] = imports

    generate_build_file(proto_files, dependency_graph, output_file)
    # 将文件中的 xxxxx 替换为 @com_google_protobuf//:timestamp_proto 


if __name__ == "__main__":
    proto_dir = "."  # Replace with your proto files directory
    output_file = "BUILD"  # Output BUILD file
    main(proto_dir, output_file)

    with open(output_file, 'r') as f:
        content = f.read()
        content = re.sub(r':timestamp_proto', r'@com_google_protobuf//:timestamp_proto', content)
    with open(output_file, 'w') as f:
        f.write(content)