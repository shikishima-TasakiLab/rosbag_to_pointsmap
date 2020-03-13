#include <sys/stat.h>
#include <sys/types.h>
#include <string>
#include <stdarg.h>
#include <iostream>

#define NO_FILE_DIR_EXIST 0
#define FILE_EXIST 1
#define DIR_EXIST 2

int filedirE(std::string path);
void makeDirectory(std::string dir_path);
std::string get_extension(std::string file_path);

/* ファイル・ディレクトリの存在確認を行う
std::string path : ファイル・ディレクトリのパス
・NO_FILE_DIR_EXIST(0):
指定したパスのファイル・ディレクトリは存在しない 
・FILE_EXIST(1):
指定したパスのファイルが存在する
・DIR_EXIST(2):
指定したパスのディレクトリが存在する */
int filedirE(std::string path)
{
    struct stat st;
    if (stat(path.c_str(), &st) != 0){
        return NO_FILE_DIR_EXIST;
    }
    else{
        mode_t m = st.st_mode;
        if(S_ISDIR(m)){
            return DIR_EXIST;
        }
        else{
            return FILE_EXIST;
        }
    }
}

/* ディレクトリを作成する
std::string dir_path : 作成するディレクトリのパス
 */
void makeDirectory(std::string dir_path)
{
    if (filedirE(dir_path) != DIR_EXIST){
        if (mkdir(dir_path.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRGRP | S_IWGRP | S_IXGRP | S_IROTH | S_IWOTH | S_IXOTH) == 0){
            std::cout << dir_path << " を作成しました" << std::endl;
        }
        else{
            std::cerr << dir_path << " の作成に失敗しました" << std::endl;
        }
    }
    else{
        std::cerr << dir_path << " は既に存在します" << std::endl;
    }
}

/* 拡張子を取得する
std::string file_path : 拡張子を取得するファイルのパス
return : std::string 拡張子
*/
std::string get_extension(std::string file_path)
{
    size_t dir_i = file_path.find_last_of("/");
    size_t ext_i = file_path.find_last_of(".");

    if (dir_i == std::string::npos) {
        return file_path.substr(ext_i, file_path.size() - ext_i);
    }
    else if (dir_i < ext_i) {
        return file_path.substr(ext_i, file_path.size() - ext_i);
    }
    else {
        return "";
    }
}

/* ファイルの存在するディレクトリのパスを取得する
std::string file_path : ディレクトリを取得するファイルのパス
return : std::string ディレクトリのパス
*/
std::string get_directory(std::string file_path)
{
    size_t dir_i = file_path.find_last_of("/");
    return file_path.substr(0UL, dir_i + 1UL);
}

/* ファイル名を取得する
std::string file_path : ファイル名を取得するファイルのパス
return : std::string ファイル名
*/
std::string get_filename(std::string file_path)
{
    size_t dir_i = file_path.find_last_of("/");
    size_t ext_i = file_path.find_last_of(".");

    if (dir_i < ext_i) {
        return file_path.substr(dir_i + 1UL, ext_i - dir_i - 1UL);
    }
    else {
        return file_path.substr(dir_i + 1UL, file_path.size() - dir_i - 1UL);
    }
}

/* パスを連結する．
path : 連結するパス．複数指定可能．
return : std::string 連結したパス．
*/
template<class... Args>
std::string path_join(const Args... path)
{
    std::string dst_path = "";
    for (std::string src_path : std::initializer_list<std::string>{path...}) {
        size_t dst_dir_i = dst_path.find_last_of("/");
        if (dst_dir_i == dst_path.length() - 1UL) {
            dst_path = dst_path.substr(0UL, dst_dir_i);
        }

        size_t src_dir_i = src_path.find_first_of("/");
        if (src_dir_i != 0UL) {
            src_path = "/" + src_path;
        }

        dst_path += src_path;
    }
    return dst_path;
}