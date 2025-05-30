#include "./headers/ui/ViewerApp.h"

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " file1.[ply|xyz] file2.[ply|xyz]\n";
    return 1;
  }
  ViewerApp app;
  app.run(argv[1], argv[2]);
  return 0;
}
