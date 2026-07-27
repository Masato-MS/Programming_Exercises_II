// ナップサック問題を解くプログラム

#include <iostream>
#include <vector>
#include <algorithm> // max関数やreverse関数を使うために必要

using namespace std;

int main() {
	int n, W; // n: アイテム数, W: ナップサック容量
	cout << "ナップサック問題を解くプログラム" << endl;
	cout << "アイテム数とナップサック容量を入力してください: " << endl;

	cout << "アイテム数 n: ";
	cin >> n; //  n: アイテム数, 

	cout << "ナップサック容量 W: ";
	cin >> W; // W: ナップサック容量

	vector<int> weight(n), value(n); // int型のベクトルを作成し、アイテムの重さと価値をそれぞれ格納する、(n)でn個（アイテム総数分）の要素を持つベクトルを作成する
    cout << "全アイテム の重さと価値を入力してください " << endl;

    for (int i = 0; i < n; ++i) {
		cout << "アイテム " << i + 1 << " の重さ：  ";
        cin >> weight[i];
        cout << "アイテム " << i + 1 << " の価値：  ";
        cin >> value[i];
    }

    // V[i][w]: i番目までのアイテムを用いて、重さw以下で得られる最大価値
    vector<vector<int>> V(n + 1, vector<int>(W + 1, 0)); //  vector<vector<int>> V(行数, vector<int>(列数, 初期値)) の文法で2次元配列を作る;
	                                                     // アイテムは0からnまであるので、Vの行数はn+1、ナップサック容量は0からWまであるので、Vの列数はW+1となる。初期値は0で埋めておく。

    for (int i = 0; i < n; ++i) {
        for (int w = 0; w <= W; ++w) {
			if (w < weight[i]) { // アイテムiの重さが現在の容量wより大きい場合、アイテムiは選べないので、前のアイテムまでの最大価値をそのまま引き継ぐ
                V[i + 1][w] = V[i][w];
            }
            else {
				V[i + 1][w] = max(V[i][w], V[i][w - weight[i]] + value[i]); // アイテムiを選ぶ場合と選ばない場合の最大価値を比較して、大きい方を選ぶ
            }
        }
    }

    cout << "最大価値： " << V[n][W] << endl;


	// 逆からたどることで、どのアイテムが選ばれたかを特定する
    int w = W;
    vector<int> selected_items;
    for (int i = n; i > 0; --i) {
        if (V[i][w] != V[i - 1][w]) {
            selected_items.push_back(i); // i番目のアイテムを選ぶ
			w -= weight[i - 1]; // 選んだアイテムの重さを引く
        }
    }

	// ベクトルの要素を反転して、アイテム番号を昇順にして表示する
	reverse(selected_items.begin(), selected_items.end()); // reverse関数でベクトルの要素を反転する

    cout << "選択されたアイテム: ";
	for (int i = 0; i < selected_items.size(); ++i) { // 選択されたアイテムの番号を順に取り出して表示する
        cout << " " << selected_items[i];
    }

    return 0;
}
