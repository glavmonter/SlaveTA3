..\nanopb-0.3.9.2-windows-x86\generator-bin\protoc.exe --nanopb_out=-v:. slave.proto
move slave.pb.h include\
move slave.pb.c src\

protoc --python_out=. slave.proto
move slave_pb2.py "Slave_TA3 Tools"

protoc --csharp_out=. slave.proto
