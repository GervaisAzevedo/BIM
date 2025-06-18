#include "./ui/ViewerApp.h"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " file1.[ply|xyz] \n";
    return 1;
  }
  ViewerApp app;
  app.run(argv[1]);
  return 0;
}
