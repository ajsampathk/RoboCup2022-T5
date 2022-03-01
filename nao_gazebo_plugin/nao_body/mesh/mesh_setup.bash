function createAndCopy () {
    echo "Creating directory /usr/share/nao_body_mesh"
    mkdir /usr/share/nao_body_mesh
    echo "Copying STL.."
    cp ./nao_robot.stl /usr/share/nao_body_mesh
    echo "Done.."
}

if [ -d "/usr/share/nao_body_mesh" ] 
then
    echo "Directory /usr/share/nao_body_mesh already exists."
    read -r -p "Remove and setup again? [y/N] " response
    case "$response" in
        [yY][eE][sS]|[yY]) 
            rm -rf /usr/share/nao_body_mesh
            createAndCopy
            ;;
        *)
            echo "Exiting.."
            ;;  
    esac 
else
    echo "Starting set up"
    createAndCopy
fi

