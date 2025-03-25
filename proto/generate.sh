for i in `ls *.proto`; do ~/nanopb/generator-bin/nanopb_generator -I . $i; done
mv *.pb.* ../src/
