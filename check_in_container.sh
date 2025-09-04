if [ ! -f /.dockerenv ]; then
    echo "Please run inside of a docker container"
    exit 1
fi
