// 移動コストと障害物を考慮したA*経路探索シミュレータ

// このプログラムでは、次の処理を行う。
// 1. 壁と移動コストを持つ二次元マップを乱数で生成する。
// 2. A*アルゴリズムを使い、総移動コストが最小になる経路を求める。
// 3. 求めた経路を画面に表示する。
// 4. ロボットRを、求めた経路に沿って1秒ごとに移動させる。


#include <iostream> // このプログラムでは、日本語表示にワイド文字を使い、wcout，wcinを使う。
#include <vector>

// queueとpriority_queueを使うために必要。
// A*では、評価値の小さい候補を優先して取り出すpriority_queueを使う。
#include <queue>

#include <random>
#include <algorithm>

// ミリ秒という時間単位chrono::millisecondsを使うために必要。
#include <chrono>

// プログラムを一時停止するthis_thread::sleep_forを使うために必要。
// ロボットを1秒ごとに動かすアニメーションで使用する。
#include <thread>

#include <iomanip>

// int型で表現できる最大値などを取得する
// numeric_limitsを使うために必要。
#include <limits>

// string、wstring、to_wstringを使うために必要。
// 日本語を含む文字列は主にwstringで扱う。
#include <string>

#include <cmath>

// system関数を使うために必要。
// Windowsではsystem("cls")でコンソール画面を消去する。
#include <cstdlib>

// 日本語など、文字の地域設定に関する機能を使うために必要。
#include <locale>
#include <clocale>

// _WIN32は、Windowsでコンパイルしている場合に自動的に定義される。
// #ifdef～#endifの中は、Windowsの場合だけコンパイルされる。
#ifdef _WIN32

// _filenoを使うために必要。
#include <io.h>

// _setmode、_O_U16TEXTを使うために必要。
// Windowsコンソールで日本語を文字化けしにくくするために使用する。
#include <fcntl.h>
#endif

using namespace std;


// ============================================================
// 座標
// ============================================================

// structは、複数の関連する変数を1つにまとめるためのもの。
// Positionは、マップ上の1つの座標を表す。
struct Position {
    int x = 0;
    int y = 0;

    // ==演算子の動作をPosition用に定義している。
    // この定義により、if (current == goal) のように座標を比較できる。
    //
    // const Position& other
    //   const : 関数内でotherを書き換えない。
    //   &     : コピーせず、元のPositionを参照するため処理が軽い。
    //
    // 関数末尾のconst
    //   この関数内で、自分自身のxやyを書き換えないことを示す。
    bool operator==(const Position& other) const {
        // xとyの両方が同じならtrueを返す。
        return x == other.x && y == other.y;
    }

    // !=演算子の動作をPosition用に定義する。
    bool operator!=(const Position& other) const {
        // *thisは「このPosition自身」を表す。

        return !(*this == other);
    }
};

// ============================================================
// 探索結果
// ============================================================

// A*探索を行った後に得られる結果を1つにまとめる構造体。
struct SearchResult {
    // ゴールまでの経路が見つかった場合はtrueになる。
    bool found = false;

    // スタートからゴールまでの座標を順番に保存する。
    // 例: {(0,0), (1,0), (1,1), ...}
    vector<Position> path;

    // 経路上のマスの移動コストを合計した値。
    // このプログラムではスタートとゴールのコストは加えない。
    int totalCost = 0;

    // ロボットが何回マス間を移動するかを表す。
    // 経路に10個の座標があれば、移動回数は9回になる。
    int stepCount = 0;
};

// ============================================================
// シミュレーション設定
// ============================================================

// マップ生成に使う設定値をまとめた構造体。
struct SimulationSettings {
    // マップの横幅。
    int width = 20;

    // マップの縦幅。
    int height = 15;

    // 各マスが壁になる確率。
    double wallRate = 0.20;

    // 通行可能なマスに設定する移動コストの最小値。
    int minimumCost = 1;

    // 通行可能なマスに設定する移動コストの最大値。
    int maximumCost = 9;
};

// constexprは「コンパイル時に決まる定数」を表す。
// ロボットの表示を切り替える間隔を1000 ms に固定する。
constexpr int ANIMATION_DELAY_MS = 1000;

// ============================================================
// 入力補助関数
// ============================================================
// 数字を入力すべき場所で文字などが入力された場合に、
// wcinのエラー状態と不正な入力を取り除く関数。
void clearInputError() {
    // 入力失敗状態を解除し、再び入力できるようにする。
    wcin.clear();

    // 現在の入力行に残っている文字を改行まで読み飛ばす。
    // numeric_limits<streamsize>::max()は、
    // 「読み飛ばす最大文字数を十分大きな値にする」という意味。
    wcin.ignore(
        numeric_limits<streamsize>::max(),
        L'\n'
    );
}

// 指定した範囲内の整数が入力されるまで、入力を繰り返す関数。
//
// const wstring& message:
//   画面に表示する入力案内。wstringは日本語を扱える文字列型。
// int minimum, maximum:
//   入力を許可する最小値と最大値。
//
// 戻り値int:
//   正しく入力された整数を呼び出し元へ返す。
int inputInteger(
    const wstring& message,
    int minimum,
    int maximum
) {
    while (true) {
        int value;

        wcout << message;

        // wcin >> valueが失敗するとfalseになる。
        // !で反転するので、整数を入力できなかった場合にif内へ入る。
        if (!(wcin >> value)) {
            wcout << L"整数を入力してください。\n";

            // 不正な入力を取り除く。
            clearInputError();

            // continueで以降の処理を行わずwhileの先頭へ戻る。
            continue;
        }

        // 入力値が許可範囲外なら、もう一度入力させる。
        if (value < minimum || value > maximum) {
            wcout
                << minimum
                << L"～"
                << maximum
                << L"の範囲で入力してください。\n";
            continue;
        }

        return value;
    }
}

// inputIntegerとほぼ同じ処理だが、こちらは小数を含むdouble型を入力する。
// 壁の生成確率を入力するときに使用する。
double inputDouble(
    const wstring& message,
    double minimum,
    double maximum
) {
    while (true) {
        double value;
        wcout << message;

        // 数値として読み取れなかった場合。
        if (!(wcin >> value)) {
            wcout << L"数値を入力してください。\n";
            clearInputError();
            continue;
        }

        // 指定した範囲に入っているか確認する。
        if (value < minimum || value > maximum) {
            wcout
                << minimum
                << L"～"
                << maximum
                << L"の範囲で入力してください。\n";
            continue;
        }

        return value;
    }
}

// ユーザーがEnterキーを押すまで画面を止める関数。
void waitForEnter() {
    wcout << L"\nEnterキーを押してください。";

    // 直前の数値入力後に残っている改行文字を捨てる。
    wcin.ignore(
        numeric_limits<streamsize>::max(),
        L'\n'
    );

    // 次の1文字、つまりEnterが入力されるまで待つ。
    wcin.get();
}


// コンソール画面を消去する関数。
void clearConsole() {
#ifdef _WIN32
    // Windowsのコマンドclsを実行して画面を消去する。
    system("cls");
#else
    // Windows以外ではANSIエスケープシーケンスで画面を消去する。
    wcout << L"\x1B[2J\x1B[H";
#endif
}

// ============================================================
// コスト付きグリッドマップ
// 0は壁、1以上は移動コスト
// ============================================================

// classは、データと、そのデータを操作する関数をまとめるためのもの。
// GridMapクラスは、マップの大きさ、各マスの値、スタート、ゴールを管理する。
class GridMap {
private:
    // マップの横幅と縦幅。
    int width = 0;
    int height = 0;

    // cells[y][x]で、y行x列のマスの値を取得できる。
    //
    // 値が0なら壁、1以上なら通行可能で、その値が移動コスト。
    vector<vector<int>> cells;

    // スタート座標とゴール座標。（初期化）
    Position start{ 0, 0 };
    Position goal{ 0, 0 };

    // targetがpositionsの中に含まれているかを調べる補助関数。
    // マップ表示時に「現在のマスが経路上か」を調べるために使う。
    bool containsPosition(
		const vector<Position>& positions, // &はコピーせず参照
        const Position& target
    ) const {
        // findはpositions.begin()からpositions.end()までtargetを探す。
        // begin()は先頭、end()は末尾のさらに次を表す。
        //
        // 見つからなかった場合はpositions.end()が返る。
        // したがって、end()と異なれば見つかったことになる。
        return find(
            positions.begin(),
            positions.end(),
            target
        ) != positions.end();
    }

public:
    // public内の関数はクラスの外部から呼び出せる。

    // 引数なしのコンストラクタ。
    // = defaultは、特別な処理を行わない標準のコンストラクタを使うという意味。
    GridMap() = default;

    // 横幅と縦幅を受け取るコンストラクタ。
    // オブジェクト作成時にresizeを呼び、マップの大きさを設定する。
    GridMap(int mapWidth, int mapHeight) {
        resize(mapWidth, mapHeight);
    }

    // マップの大きさを変更し、マップ内容を初期化する関数。
    void resize(int mapWidth, int mapHeight) {
        width = mapWidth;
        height = mapHeight;

        // assignはvectorの要素数と初期値をまとめて設定する。
        //
        // 外側のvectorをheight個用意し、
        // 各行にはint型の要素をwidth個用意する。
        // 全マスを初期値1にする。
        cells.assign(
            height,
            vector<int>(width, 1)
        );

        // スタートは左上の(0, 0)に設定する。
        start = Position{ 0, 0 };

        // ゴールは右下に設定する。
        goal = Position{ width - 1, height - 1 };
    }

    // getから始まる関数は、private変数の値を外部へ返すための関数。
    // 末尾のconstは、関数内でGridMapの内容を変更しないことを示す。
    int getWidth() const {
        return width;
    }

    int getHeight() const {
        return height;
    }

    Position getStart() const {
        return start;
    }

    Position getGoal() const {
        return goal;
    }

    // スタート座標を変更する関数。
    void setStart(const Position& newStart) {
        start = newStart;

        // 新しいスタートがマップ内にあり、そのマスが壁なら、
        // 通行可能なコスト1へ変更する。
        if (isInside(start.x, start.y) &&
            cells[start.y][start.x] == 0) {
            cells[start.y][start.x] = 1;
        }
    }

    // ゴール座標を変更する関数。
    void setGoal(const Position& newGoal) {
        goal = newGoal;

        // ゴールが壁にならないようにする。
        if (isInside(goal.x, goal.y) &&
            cells[goal.y][goal.x] == 0) {
            cells[goal.y][goal.x] = 1;
        }
    }

    // 指定した座標がマップの範囲内かを判定する。
    bool isInside(int x, int y) const {
        // xが0以上width未満、かつyが0以上height未満ならtrue。
        return
            x >= 0 && x < width &&
            y >= 0 && y < height;
    }

    // 指定した座標が壁かを判定する。
    bool isWall(int x, int y) const {
        // マップの外側は移動できないため、壁として扱う。
        if (!isInside(x, y)) {
            return true;
        }

        // 値0のマスを壁とする。
        return cells[y][x] == 0;
    }

    // 指定した座標を通行できるかを判定する。
    bool isWalkable(int x, int y) const {
        // マップ内にあり、値が1以上なら通行可能。
        return
            isInside(x, y) &&
            cells[y][x] > 0;
    }

    // 指定したマスの移動コストを返す。
    int getCost(int x, int y) const {
        // マップ外なら安全のため0を返す。
        if (!isInside(x, y)) {
            return 0;
        }

        return cells[y][x];
    }

    // 指定したマスの値を変更する。
    void setCost(int x, int y, int cost) {
        // マップ内の場合だけ値を変更する。
        if (isInside(x, y)) {
            cells[y][x] = cost;
        }
    }

    // マップ内に存在する通行可能マスのうち、最小コストを求める。
    // A*の推定コストhを計算するときに使用する。
    int getMinimumPassableCost() const {
        // 最初は非常に大きな値を入れておく。
        int minimum = numeric_limits<int>::max();

        // 範囲for文で、cellsの各行を順番に取り出す。
        //
        // const auto& row:
        //   autoは型を自動で決定する。
        //   この場合rowはconst vector<int>&になる。
        //   &を付けることで行全体をコピーせず参照する。
        for (const auto& row : cells) {
            // 行の中の値を1つずつ取り出す。
            for (int value : row) {
                // 0は壁なので除外し、現在のminimumより小さければ更新する。
                if (value > 0 && value < minimum) {
                    minimum = value;
                }
            }
        }

        // 通行可能なマスが1つも見つからなかった場合の安全処理。
        if (minimum == numeric_limits<int>::max()) {
            return 1;
        }

        return minimum;
    }

    // マップ内に壁が何個あるかを数える。
    int countWalls() const {
        int count = 0;

        for (const auto& row : cells) {
            for (int value : row) {
                if (value == 0) {
                    ++count;
                }
            }
        }

        return count;
    }

    // 壁と移動コストを乱数で生成する関数。
    //
    // mt19937& generator:
    //   乱数を作るための乱数生成器を参照で受け取る。
    //   mt19937は、一般的に使われる高品質な疑似乱数生成器。
    //
    // wallRate:
    //   壁になる確率。例: 0.20なら20%。
    //
    // minimumCost～maximumCost:
    //   通行可能マスに設定する移動コストの範囲。
    void generateRandom(
        mt19937& generator,
        double wallRate,
        int minimumCost,
        int maximumCost
    ) {
        // bernoulli_distributionはtrueまたはfalseを返す乱数。
        // wallRateの確率でtrueを返す。
        bernoulli_distribution wallDistribution(wallRate);

        // 指定範囲の整数を同じ確率で発生させる乱数。
        // 例えば1～9なら、1から9のいずれかを返す。
        uniform_int_distribution<int> costDistribution(
            minimumCost,
            maximumCost
        );

        // 全マスを順番に処理する二重for文。
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                // trueになった場合、このマスを壁0にする。
                if (wallDistribution(generator)) {
                    cells[y][x] = 0;
                }
                else {
                    // 壁でない場合は、乱数で移動コストを設定する。
                    cells[y][x] = costDistribution(generator);
                }
            }
        }

        // スタートとゴールは必ず通行可能にする。
        // 表示上はSとGになるが、内部ではminimumCostが保存されている。
        cells[start.y][start.x] = minimumCost;
        cells[goal.y][goal.x] = minimumCost;

        // 開始直後と終了直前が完全に塞がれにくいようにする。
        //
        // dxとdyの同じ番号を組み合わせることで、
        // 右、左、下、上の4方向を表す。
        //
        // i=0: (x+1, y)   右
        // i=1: (x-1, y)   左
        // i=2: (x, y+1)   下
        // i=3: (x, y-1)   上
        const int dx[4] = { 1, -1, 0, 0 };
        const int dy[4] = { 0, 0, 1, -1 };

        // スタート周辺の4方向を調べる。
        for (int i = 0; i < 4; ++i) {
            int sx = start.x + dx[i];
            int sy = start.y + dy[i];

            // 最初に見つけた壁を通行可能マスへ変更する。
            if (isInside(sx, sy) &&
                cells[sy][sx] == 0) {
                cells[sy][sx] = costDistribution(generator);

                break;
            }
        }

        // ゴール周辺についても同じ処理を行う。
        for (int i = 0; i < 4; ++i) {
            int gx = goal.x + dx[i];
            int gy = goal.y + dy[i];

            if (isInside(gx, gy) &&
                cells[gy][gx] == 0) {
                cells[gy][gx] = costDistribution(generator);
                break;
            }
        }
    }

    // 1本の経路に含まれる移動コストの合計を計算する。
    int calculatePathCost(
        const vector<Position>& path
    ) const {
        int total = 0;

        // size_tは、vectorの要素数や添字に使われる符号なし整数型。
        //
        // i=1から開始するため、path[0]のスタートは加算しない。
        // i+1 < path.size()とするため、最後のゴールも加算しない。
        for (size_t i = 1; i + 1 < path.size(); ++i) {
            total += getCost(path[i].x, path[i].y);
        }

        return total;
    }

    // マップをコンソール画面へ表示する関数。
    //
    // path = {}:
    //   経路を渡さなかった場合は、空のvectorを使う。
    //
    // const Position* robot = nullptr:
    //   Positionへのポインタを受け取る。
    //   nullptrは「ロボット位置が指定されていない」ことを表す。
    //   ロボット移動中だけ実際の座標のアドレスを渡す。
    //
    // title = L"マップ":
    //   タイトルを省略した場合は「マップ」と表示する。
    //
    // L"..."はワイド文字列で、日本語をwcoutに表示するときに使う。
    void display(
        const vector<Position>& path = {},
        const Position* robot = nullptr,
        const wstring& title = L"マップ"
    ) const {
        wcout << L"\n" << title << L"\n\n";

        // 横方向の座標番号を表示する。
        wcout << L"    ";
        for (int x = 0; x < width; ++x) {
            // setw(4)は、次に表示する値の幅を4文字分にそろえる。
            wcout << setw(4) << x;
        }
        wcout << L"\n";

        // y方向に1行ずつ表示する。
        for (int y = 0; y < height; ++y) {
            // 左端にy座標を表示する。
            wcout << setw(3) << y << L" ";

            // 1行の中をx方向へ表示する。
            for (int x = 0; x < width; ++x) {
                // 現在表示しているマスの座標。
                Position current{ x, y };

                // このマスに表示する文字列。
                wstring text;

                // robotがnullptrでなく、現在座標とロボット座標が同じ場合。
                //
                // robotはPosition*なので、*robotと書くことで
                // ポインタが指しているPosition本体を取得する。
                if (robot != nullptr && current == *robot) {
                    text = L"R";
                }
                else if (current == start) {
                    text = L"S";
                }
                else if (current == goal) {
                    text = L"G";
                }
                else if (cells[y][x] == 0) {
                    text = L"###";
                }
                else if (containsPosition(path, current)) {
                    // to_wstringは数値をwstringへ変換する。
                    // 例: コスト5なら「*5」という文字列を作る。
                    text = L"*" + to_wstring(cells[y][x]);
                }
                else {
                    // 通常マスはコストの数字だけを表示する。
                    text = to_wstring(cells[y][x]);
                }

                // 各マスを4文字幅で表示して、縦横をそろえる。
                wcout << setw(4) << text;
            }

            wcout << L"\n";
        }

        // 記号の説明を表示する。
        wcout << L"\n";
        wcout << L"S：スタート　G：ゴール　R：ロボット\n";
        wcout << L"###：壁　1～9：移動コスト　*数字：選択経路\n";
    }

};

// ============================================================
// A*探索
// ============================================================

// A*アルゴリズムを使って、総移動コストが最小になる経路を求めるクラス。
class AStarPathFinder {
private:
    // A*が探索候補として扱う1つのノードを表す。
    struct Node {
        // このノードが表すマップ上の座標。
        Position position;

        // g: スタートからこの座標までに実際にかかった移動コスト。
        int g = 0;

        // h: この座標からゴールまでにかかると予想したコスト。
        // 実際の残りコストではなく、推定値である。
        int h = 0;

        // f: g+h。
        // A*は、このfが小さいノードから探索する。
        int f = 0;
    };

    // priority_queue内で、どのNodeを先に取り出すかを決める比較処理。
    struct NodeCompare {
        // priority_queueは通常、大きい値を先に取り出す。
        // 今回はfが小さいNodeを優先したいため、比較の向きを逆にする。
        bool operator()(const Node& left, const Node& right) const {
            if (left.f != right.f) {
                // left.fの方が大きい場合にtrueを返し、
                // leftの優先度を低くする。
                return left.f > right.f;
            }

            // fが同じ場合はhが小さい、つまりゴールに近い方を優先する。
            return left.h > right.h;
        }
    };

    // 現在位置からゴールまでの推定コストhを求める関数。
    //
    // 上下左右にしか移動しないため、
    // x方向の差とy方向の差を足すマンハッタン距離を使う。
    int heuristic(
        const Position& current,
        const Position& goal,
        int minimumCost
    ) const {

        int manhattanDistance =
            abs(current.x - goal.x) +
            abs(current.y - goal.y);

        // 1マス進む最低コストを掛けることで、
        // ゴールまでに最低でも必要になるコストを推定する。
        //
        // 実際のコストを大きく見積もりすぎないため、
        // A*で最小コスト経路を求めることができる。
        return manhattanDistance * minimumCost;
    }

public:
    // 実際にA*探索を行う関数。
    //
    // const GridMap& map:
    //   マップ全体をコピーせず参照で受け取る。
    //   constを付けているため、この関数内ではマップを書き換えない。
    //
    // 戻り値SearchResult:
    //   経路が見つかったか、経路、総コスト、移動回数を返す。
    SearchResult search(const GridMap& map) const {
        // 探索結果を保存する変数。
        // 初期状態ではfound=false、コストや回数は0。
        SearchResult result;

        // 何度も関数を呼ばなくてよいよう、必要な情報を変数へ保存する。
        const int width = map.getWidth();
        const int height = map.getHeight();
        const Position start = map.getStart();
        const Position goal = map.getGoal();

        // マップ内で最も小さい通行コスト。
        // 推定コストhの計算に使う。
        const int minimumCost = map.getMinimumPassableCost();

        // 到達していないマスのgを表すための十分大きな値。
        // 最大値そのものでは、加算時にオーバーフローしやすいため4で割る。
        const int infinity = numeric_limits<int>::max() / 4;

        // 各マスまでに現在判明している最小の実コストgを保存する。
        //
        // 最初は、全マスを「まだ到達していない」とするためinfinityで初期化。
        vector<vector<int>> gScore(
            height,
            vector<int>(width, infinity)
        );

        // 各マスへ、どのマスから来たかを保存する二次元配列。
        //
        // 初期値(-1,-1)は「まだ移動元が設定されていない」という印。
        vector<vector<Position>> parent(
            height,
            vector<Position>(
                width,
                Position{ -1, -1 }
            )
        );

        // すでに探索を確定したマスを記録する。
        // falseなら未確定、trueなら探索済み。
        vector<vector<bool>> closed(
            height,
            vector<bool>(width, false)
        );

        // 次に調べる候補を保存する優先度付きキュー。
        //
        // 第1引数Node:
        //   保存するデータ型。
        //
        // 第2引数vector<Node>:
        //   内部でデータを保存する入れ物。
        //
        // 第3引数NodeCompare:
        //   Nodeの優先順位を決める比較方法。
        priority_queue<
            Node,
            vector<Node>,
            NodeCompare
        > openQueue;

        // スタートからゴールまでの推定コストhを計算する。
        int startH = heuristic(
            start,
            goal,
            minimumCost
        );

        // スタート地点までにかかった実コストgは0。
        gScore[start.y][start.x] = 0;

        // スタート地点を最初の探索候補としてキューへ追加する。
        //
        // Node{start, 0, startH, startH}は、
        // position=start、g=0、h=startH、f=0+startHを表す。
        openQueue.push(
            Node{
                start,
                0,
                startH,
                startH
            }
        );

        // 現在位置から移動する4方向。
        // dx[i]とdy[i]を同じiで組み合わせて使用する。
        const int dx[4] = { 1, -1, 0, 0 };
        const int dy[4] = { 0, 0, 1, -1 };

        // 探索候補が残っている間、処理を繰り返す。
        // empty()は中身が空ならtrueを返すため、!を付けて
        // 「空ではない間」という条件にしている。
        while (!openQueue.empty()) {
            // top()で最も優先度の高いNodeを取得する。
            Node currentNode = openQueue.top();

            // 取得したNodeをキューから削除する。
            openQueue.pop();

            // Nodeから座標だけを取り出す。
            Position current = currentNode.position;

            // 同じ座標が複数回キューに入る場合がある。
            // すでに探索確定済みなら処理せず次へ進む。
            if (closed[current.y][current.x]) {
                continue;
            }

            // このマスを探索済みにする。
            closed[current.y][current.x] = true;

            // ゴールを取り出した時点で、最小コスト経路が確定する。
            if (current == goal) {
                result.found = true;
                break;
            }

            // 現在位置の右、左、下、上を順番に調べる。
            for (int i = 0; i < 4; ++i) {
                // 次に調べる隣接マスの座標を作る。
                Position next{
                    current.x + dx[i],
                    current.y + dy[i]
                };

                // マップ外または壁なら移動できないため無視する。
                if (!map.isWalkable(next.x, next.y)) {
                    continue;
                }

                // すでに探索確定済みのマスなら無視する。
                if (closed[next.y][next.x]) {
                    continue;
                }

                // 現在位置までのコストに、
                // 次のマスへ入るためのコストを加える。
                //
                // この経路でnextへ進んだ場合の仮のgを表す。
                int tentativeG =
                    gScore[current.y][current.x] +
                    map.getCost(next.x, next.y);

                // すでに記録されているnextまでのコスト以下でなければ、
                // この新しい経路を採用する必要がない。
                if (tentativeG >= gScore[next.y][next.x]) {
                    continue;
                }

                // より小さいコストの経路が見つかったので、
                // nextまでの最小コストgを更新する。
                gScore[next.y][next.x] = tentativeG;

                // 経路復元のため、nextへcurrentから来たことを記録する。
                parent[next.y][next.x] = current;

                // nextからゴールまでの推定コストhを求める。
                int nextH = heuristic(
                    next,
                    goal,
                    minimumCost
                );

                // nextを新しい探索候補として追加する。
                // fはtentativeG + nextH。
                openQueue.push(
                    Node{
                        next,
                        tentativeG,
                        nextH,
                        tentativeG + nextH
                    }
                );
            }
        }

        // ゴールまでの経路が見つかった場合、実際の経路を復元する。
        if (result.found) {
            // ゴールから親を逆向きにたどる。
            Position current = goal;

            while (current != start) {
                // push_backはvectorの末尾へ要素を追加する。
                result.path.push_back(current);

                // currentを、そのマスへ来る直前の座標に変更する。
                current = parent[current.y][current.x];
            }

            // whileはスタートを追加する前に終了するため、最後に追加する。
            result.path.push_back(start);

            // 現在は「ゴール→…→スタート」の逆順なので、
            // reverseで「スタート→…→ゴール」に並べ替える。
            reverse(
                result.path.begin(),
                result.path.end()
            );

            // path.size()の型はsize_tなので、intへ明示的に変換する。
            // static_cast<int>(値)は、型をintへ変換する書き方。
            //
            // 座標数がN個なら、マス間の移動回数はN-1回。
            result.stepCount =
                static_cast<int>(result.path.size()) - 1;

            // 復元した経路の総移動コストを計算する。
            result.totalCost =
                map.calculatePathCost(result.path);
        }

        // 探索結果を呼び出し元へ返す。
        return result;
    }
};

// ============================================================
// ロボット
// ============================================================

// 経路に沿って動くロボットの現在状態を管理するクラス。
class Robot {
private:
    // 現在の座標。
    Position position;

    // スタート後、現在までに通過したマスのコスト合計。
    int accumulatedCost = 0;

    // 現在までに何回マス間を移動したか。
    int moveCount = 0;

public:
    // ロボットをスタート地点へ戻し、記録を0にする。
    void reset(const Position& start) {
        position = start;
        accumulatedCost = 0;
        moveCount = 0;
    }

    // ロボットをnextへ1マス移動させる。
    void moveTo(
        const Position& next,
        const GridMap& map
    ) {
        // 現在位置を更新する。
        position = next;

        // ゴール地点のコストは表示上Gになっており、
        // このプログラムでは総コストに含めない設定なので加算しない。
        if (next != map.getGoal()) {
            accumulatedCost += map.getCost(next.x, next.y);
        }

        // 1マス移動したため移動回数を1増やす。
        ++moveCount;
    }

    // 現在位置を返す。
    Position getPosition() const {
        return position;
    }

    // 現在までの累積コストを返す。
    int getAccumulatedCost() const {
        return accumulatedCost;
    }

    // 現在までの移動回数を返す。
    int getMoveCount() const {
        return moveCount;
    }
};

// ============================================================
// シミュレータ
// ============================================================

// メニュー表示、マップ生成、探索、アニメーションなど、
// プログラム全体の流れを管理するクラス。
class Simulator {
private:
    // 現在のマップ設定。
    SimulationSettings settings;

    // 現在生成されているマップ。
    GridMap map;

    // A*探索を行うオブジェクト。
    AStarPathFinder aStarPathFinder;

    // アニメーションで動かすロボット。
    Robot robot;

    // 乱数生成器。
    // 1つの生成器を繰り返し使い、マップの乱数を作る。
    mt19937 generator;

    // 最後に行ったA*探索の結果。
    // メニュー3で探索した結果をメニュー2や4でも再利用する。
    SearchResult lastAStarResult;

    // 使用可能なマップが生成済みか。
    bool mapReady = false;

    // A*の経路が計算済みか。
    bool aStarResultReady = false;

    // 経路が存在するマップができるまで何回生成したか。
    int generationAttempts = 0;

    // 画面上部にプログラム名を表示する。
    void printTitle() const {
        wcout << L"============================================\n";
        wcout << L" コスト付きランダムマップ A*経路探索\n";
        wcout << L" 自律移動ロボットシミュレータ\n";
        wcout << L"============================================\n";
    }

    // 操作メニューを表示する。
    void printMenu() const {
        wcout << L"\nメニュー\n";
        wcout << L"1：ランダムマップを生成\n";
        wcout << L"2：現在のマップを表示\n";
        wcout << L"3：A*で最小コスト経路を探索\n";
        wcout << L"4：ロボットの移動を再生\n";
        wcout << L"5：設定を変更\n";
        wcout << L"0：終了\n";
    }

    // 前回の探索結果を初期状態へ戻す。
    void resetResults() {
        // SearchResult{}は、SearchResultを初期値で作り直す書き方。
        lastAStarResult = SearchResult{};
        aStarResultReady = false;
    }

    // 現在のマップ生成設定を表示する。
    void displaySettings() const {
        wcout << L"\n現在の設定\n";
        wcout << L"マップ横幅　　　：" << settings.width << L"\n";
        wcout << L"マップ縦幅　　　：" << settings.height << L"\n";
        wcout << L"壁の生成確率　　："
            // fixedは小数を通常の小数表記にする。
            // setprecision(1)は小数点以下を1桁にする。
            << fixed << setprecision(1)
            // 内部では0.20なので、表示時に100倍して20.0%にする。
            << settings.wallRate * 100.0
            << L" %\n";
        wcout << L"最小移動コスト　：" << settings.minimumCost << L"\n";
        wcout << L"最大移動コスト　：" << settings.maximumCost << L"\n";
    }

    // スタートからゴールまで到達できるランダムマップを生成する。
    //
    // 戻り値:
    //   生成成功ならtrue、2000回試しても失敗したらfalse。
    bool createReachableMap() {
        // 無限に生成を繰り返さないよう、最大回数を設定する。
        const int maximumAttempts = 2000;

        // 生成試行回数を0へ戻す。
        generationAttempts = 0;

        // 現在の設定に合わせてマップの大きさを変更する。
        map.resize(settings.width, settings.height);

        // 経路が存在するマップができるまで生成を繰り返す。
        while (generationAttempts < maximumAttempts) {
            ++generationAttempts;

            // 壁とコストを乱数で生成する。
            map.generateRandom(
                generator,
                settings.wallRate,
                settings.minimumCost,
                settings.maximumCost
            );

            // 生成したマップでA*を行い、経路があるか確認する。
            // この探索はマップ確認用なので、結果はtestResultに一時保存する。
            SearchResult testResult =
                aStarPathFinder.search(map);

            // 経路が見つかったマップなら採用する。
            if (testResult.found) {
                mapReady = true;

                // 新しいマップになったため、以前の探索結果は無効にする。
                resetResults();

                return true;
            }
        }

        // 最大回数まで経路のあるマップを生成できなかった。
        mapReady = false;
        return false;
    }

    // メニュー1が選ばれたときの処理。
    void generateMapMenu() {
        clearConsole();
        printTitle();
        displaySettings();

        wcout << L"\n経路が存在するランダムマップを生成します。\n";

        // createReachableMapがfalseなら生成失敗。
        if (!createReachableMap()) {
            wcout << L"\nマップを生成できませんでした。\n";
            wcout << L"壁の確率を下げてください。\n";
            waitForEnter();
            return;
        }

        // {}は空の経路、nullptrはロボット位置なしを表す。
        map.display({}, nullptr, L"生成したランダムマップ");

        wcout << L"\n生成試行回数：" << generationAttempts << L"回\n";
        wcout << L"壁の数　　　　：" << map.countWalls() << L"個\n";

        waitForEnter();
    }

    // メニュー2が選ばれたときの処理。
    // 現在のマップを再表示する。
    void displayMapMenu() const {
        clearConsole();
        printTitle();

        // まだマップを生成していない場合は処理できない。
        if (!mapReady) {
            wcout << L"\n先にランダムマップを生成してください。\n";
            waitForEnter();
            return;
        }

        // A*探索済みなら、選択された経路も表示する。
        if (aStarResultReady) {
            map.display(
                lastAStarResult.path,
                nullptr,
                L"A*の最小コスト経路"
            );
        }
        else {
            // 未探索なら経路なしのマップを表示する。
            map.display({}, nullptr, L"現在のマップ");
        }

        waitForEnter();
    }

    // A*探索結果を文字で表示する。
    void printSearchResult(
        const wstring& name,
        const SearchResult& result
    ) const {
        wcout << L"\n" << name << L"\n";

        // 条件演算子「条件 ? 値1 : 値2」を使っている。
        // foundがtrueなら「成功」、falseなら「失敗」を表示する。
        wcout << L"経路発見　　　　："
            << (result.found ? L"成功" : L"失敗")
            << L"\n";

        // 経路がない場合は、移動回数やコストを表示せず終了する。
        if (!result.found) {
            return;
        }

        wcout << L"移動回数　　　　：" << result.stepCount << L"回\n";
        wcout << L"総移動コスト　　：" << result.totalCost << L"\n";
    }

    // メニュー3が選ばれたときの処理。
    // 現在のマップに対してA*探索を行う。
    void runAStarMenu() {
        clearConsole();
        printTitle();

        if (!mapReady) {
            wcout << L"\n先にランダムマップを生成してください。\n";
            waitForEnter();
            return;
        }

        // A*探索を実行し、結果を保存する。
        lastAStarResult = aStarPathFinder.search(map);

        // 探索を実行済みであることを記録する。
        aStarResultReady = true;

        if (!lastAStarResult.found) {
            wcout << L"\nゴールまでの経路は存在しません。\n";
            waitForEnter();
            return;
        }

        // 経路を*数字で表示する。
        map.display(
            lastAStarResult.path,
            nullptr,
            L"A*で求めた最小コスト経路"
        );

        // 移動回数と総移動コストを表示する。
        printSearchResult(
            L"A*探索結果",
            lastAStarResult
        );

        waitForEnter();
    }

    // メニュー4が選ばれたときの処理。
    // A*で求めた経路に沿って、ロボットRを1マスずつ表示する。
    void animateMenu() {
        // マップがなければ再生できない。
        if (!mapReady) {
            clearConsole();
            printTitle();
            wcout << L"\n先にランダムマップを生成してください。\n";
            waitForEnter();
            return;
        }

        // ユーザーがメニュー3を実行していなくても、
        // メニュー4を選べば自動的にA*探索を行う。
        if (!aStarResultReady) {
            lastAStarResult = aStarPathFinder.search(map);

            // 経路が見つかった場合だけtrueになる。
            aStarResultReady = lastAStarResult.found;
        }

        // 経路がない、またはpathが空なら再生できない。
        if (!aStarResultReady || lastAStarResult.path.empty()) {
            clearConsole();
            printTitle();
            wcout << L"\n再生できる経路がありません。\n";
            waitForEnter();
            return;
        }

        // ロボットをスタート地点へ戻す。
        robot.reset(map.getStart());

        // 経路の座標を先頭から順番に処理する。
        for (size_t i = 0; i < lastAStarResult.path.size(); ++i) {
            // i=0はスタート地点なので、まだ移動しない。
            // i=1以降で次のマスへ進める。
            if (i > 0) {
                robot.moveTo(
                    lastAStarResult.path[i],
                    map
                );
            }

            // 1コマごとに画面を消し、現在の状態を描き直す。
            clearConsole();
            printTitle();

            // 現在位置を取得する。
            Position current = robot.getPosition();

            // &currentはcurrent変数のアドレスを表す。
            // displayはPosition*を受け取り、その場所をRとして表示する。
            map.display(
                lastAStarResult.path,
                &current,
                L"ロボット移動中"
            );

            wcout << L"\n現在位置　　　　：("
                << current.x
                << L", "
                << current.y
                << L")\n";

            wcout << L"移動回数　　　　："
                << robot.getMoveCount()
                << L" / "
                << lastAStarResult.stepCount
                << L"\n";

            wcout << L"現在までのコスト："
                << robot.getAccumulatedCost()
                << L" / "
                << lastAStarResult.totalCost
                << L"\n";

            // この処理を指定時間だけ停止する。
            // chrono::milliseconds(1000)は1000ミリ秒。
            this_thread::sleep_for(
                chrono::milliseconds(
                    ANIMATION_DELAY_MS
                )
            );
        }

        wcout << L"\nゴールに到達しました。\n";
        waitForEnter();
    }

    // メニュー5が選ばれたときの処理。
    // マップの大きさ、壁の確率、コスト範囲を変更する。
    void settingsMenu() {
        clearConsole();
        printTitle();
        displaySettings();

        wcout << L"\n新しい設定を入力してください。\n";

        // inputIntegerを使うため、範囲外や文字入力を防止できる。
        settings.width = inputInteger(
            L"マップの横幅（8～30）：",
            8,
            30
        );

        settings.height = inputInteger(
            L"マップの縦幅（8～25）：",
            8,
            25
        );

        // ユーザーには百分率0～40として入力してもらう。
        double wallPercent = inputDouble(
            L"壁の生成確率（0～40 %）：",
            0.0,
            40.0
        );

        // 例: 20%を内部で使う0.20へ変換する。
        settings.wallRate = wallPercent / 100.0;

        settings.minimumCost = inputInteger(
            L"最小移動コスト（1～9）：",
            1,
            9
        );

        // 最大コストは、最小コスト未満にならないようにする。
        settings.maximumCost = inputInteger(
            L"最大移動コスト（最小値～9）：",
            settings.minimumCost,
            9
        );

        // 設定が変わったため、以前のマップと探索結果は使用できない。
        mapReady = false;
        resetResults();

        wcout << L"\n設定を変更しました。\n";
        wcout << L"新しい設定でマップを生成してください。\n";
        waitForEnter();
    }

public:
    // Simulatorオブジェクトを作成したときに呼ばれるコンストラクタ。
    Simulator()
        // settingsの初期値を使ってGridMapを作成する。
        : map(settings.width, settings.height),

        // random_device{}()から初期値を受け取り、
        // mt19937乱数生成器を初期化する。
        generator(random_device{}()) {
    }

    // シミュレータ全体を実行する関数。
    void run() {
        // trueの間はメニューを繰り返す。
        bool running = true;

        while (running) {
            clearConsole();
            printTitle();
            printMenu();

            // 0～5の番号を入力させる。
            int choice = inputInteger(
                L"\n番号を選択してください：",
                0,
                5
            );

            // switchはchoiceの値に応じて実行する処理を切り替える。
            switch (choice) {
            case 1:
                generateMapMenu();
                // breakがないと次のcaseまで続けて実行されるため、
                // 各caseの最後にbreakを書く。
                break;

            case 2:
                displayMapMenu();
                break;

            case 3:
                runAStarMenu();
                break;

            case 4:
                animateMenu();
                break;

            case 5:
                settingsMenu();
                break;

            case 0:
                // whileの条件をfalseにし、メニューを終了する。
                running = false;
                break;
            }
        }

        clearConsole();
        wcout << L"シミュレータを終了しました。\n";
    }
};


// ============================================================
// メイン関数
// ============================================================

// main関数は、プログラムを実行したときに最初に呼ばれる。
int main() {
#ifdef _WIN32
    // Windowsコンソールの出力モードをUTF-16へ変更する。
    // これにより、wcoutで表示する日本語が文字化けしにくくなる。
    //
    // stdoutは標準出力、つまり画面への出力。
    _setmode(
        _fileno(stdout),
        _O_U16TEXT
    );

    // stdinは標準入力、つまりキーボードからの入力。
    // wcinで日本語やワイド文字を扱えるようにする。
    _setmode(
        _fileno(stdin),
        _O_U16TEXT
    );

#else
    // Windows以外では、現在の環境に合わせた文字設定を使用する。
    setlocale(LC_ALL, "");

#endif

    // Simulatorクラスのオブジェクトを作成する。
    // この時点でSimulatorのコンストラクタが呼ばれる。
    Simulator simulator;

    // メニューを開始し、シミュレータ全体を実行する。
    simulator.run();

    return 0;
}
