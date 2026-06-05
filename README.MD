# Depth Cone Map

## Istruzioni per la compilazione
Sono necessarie le seguenti dipendenze di sistema:
- ros-humble-gtsam
- ros-humble-cv-bridge
- libopencv-dev
- libeigen3-dev
- openmp (questo dovrebbe venire gratis con faiss) \
`sudo apt install ros-humble-gtsam ros-humble-cv-bridge libopencv-dev libeigen3-dev` \
È necessaria anche la libreria `faiss`; ci sono 2 vie per installarla: 
- compilare da sorgente, si rimanda al repo ufficiale di faiss [istruzioni per l'installazione](https://github.com/facebookresearch/faiss/blob/main/INSTALL.md)
- usare vcpkg
su x86 ho usato vcpkg, su jetson l'ho compilato da sorgente. Se si usa vcpkg c'è da aggiungere il flag `-DCMAKE_TOOLCHAIN_FILE=/path/vcpkg/scripts/buildsystems/vcpkg.cmake` a colcon (suggerisco di creare il file `colcon_defaults.yaml` nella root del proprio workspace).

## TF
Per trasformare i coni questo nodo usa tf2. È necessario che il frame `map` sia collegato al frame `zed_left_camera_optical_frame` (o comunque al frame della telecamera SINISTRA).

## Lanciare il nodo
Non riesco a far funzionare l'autocompletamento nel terminale per i file .launch. Comunque sono presenti `launch_node.py` che lancia il nodo all'interno di un component container e `launch_executable.py` che lo lancia con un eseguibile standalone. I parametri necessari sono presenti in `config/parameters.yaml` mentre i topic sono rimappati nei file .launch.
