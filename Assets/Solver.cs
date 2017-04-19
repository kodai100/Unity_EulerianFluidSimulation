using UnityEngine;

public class Solver : MonoBehaviour {

    #region Public
    public int N = 200;              // グリッド解像度
    public float dt = 0.1f;          // タイムステップ幅
    public float diff = 0.000001f;   // 濃度の拡散係数 -> 大きい値:拡散しにくい
    public float visc = 0.00002f;    // 粘性係数
    public float decay = 0.99f;      // 減衰の速さ
    public float force = 1.0f;       // マウス入力で加算される外力の大きさ
    public float source = 40;        // マウス入力で加算される密度の大きさ
    public bool pause = false;	     // 一時停止
    
    public int winW = 1000;          // レンダリングウィンドウの横幅
    public int winH = 1000;          // 縦幅

    public bool color = true;        // カラーリング
    #endregion Public

    #region Private
    private float[] u, v, dens;
    private float[] u_prev, v_prev, dens_prev;

    private int bufferSize;         // 配列長 (N+2)x(N+2)
    private bool pressed = false;   // マウスが押されているかどうか
    private int mx, my;             // 現在のマウス座標
    private int lmx, lmy;           // マウスダウン時のマウス座標

    private Texture2D texture;      // 結果テクスチャ
    #endregion Private

    #region MonoBehaviour

    void Start () {
        texture = new Texture2D(N+2, N+2);
        bufferSize = (N + 2) * (N + 2);

        InitArrays();
        ClearData();
    }
    
	void Update () {
        if (pause) return;

        // prev配列の初期化
        for (int i = 0, size = (N + 2) * (N + 2); i < size; ++i) {
            dens_prev[i] = u_prev[i] = v_prev[i] = 0f;
        }

        Mouse();
        if(pressed) Inject();

        CalcVelocity();
        CalcDensity();

        Display();
    }

    void OnGUI() {
        GUI.DrawTexture(new Rect(0, 0, 1000, 1000), texture);
    }

    #endregion MonoBehaviour

    #region Funcs

    void InitArrays() {
        u = new float[bufferSize];
        v = new float[bufferSize];
        u_prev = new float[bufferSize];
        v_prev = new float[bufferSize];
        dens = new float[bufferSize];
        dens_prev = new float[bufferSize];
    }

    void Mouse() {
        if (Input.GetMouseButtonDown(0)) {
            MouseDown((int)Input.mousePosition.x, (int)Input.mousePosition.y);
        }

        MouseDrag((int)Input.mousePosition.x, (int)Input.mousePosition.y);

        if (Input.GetMouseButtonUp(0)) {
            pressed = false;
        }
    }

    void Display() {
        if (color) {
            for (int i = 0; i < bufferSize; i++) {
                texture.SetPixel(i % (N + 2), ((N + 2) - 1) - i / (N + 2), Color.HSVToRGB(dens[i], 1f, 1f));
            }
        } else {
            for (int i = 0; i < bufferSize; i++) {
                texture.SetPixel(i % (N + 2), ((N + 2) - 1) - i / (N + 2), new Color(dens[i], dens[i], dens[i]));
            }
        }
        
        texture.Apply();
    }

    int CalcIndex(int i, int j) {
        return i + (N + 2) * j;
    }

    void ClearData() {
        for (int i = 0; i < bufferSize; i++) {
            u[i] = v[i] = u_prev[i] = v_prev[i] = dens[i] = dens_prev[i] = 0f;
        }
    }

    void MouseDown(int x, int y) {
        pressed = true;
        lmx = mx = x;
        lmy = my = y;
    }

    void MouseDrag(int x, int y) {
        mx = x;
        my = y;
    }
    
    void Inject(){
        float ndx = 0, ndy = 0;
        float dx = mx - lmx, dy = my - lmy;
        float len = Mathf.Sqrt(dx * dx + dy * dy);
        if (len > 5e-5f) {
            ndx = dx / len; ndy = -dy / len;
        }

        float wn = (float)winW / (N + 1);
        float hn = (float)winH / (N + 1);

        int nsteps = (int)(len / ((wn + hn) / 2));
        if (nsteps == 0) nsteps = 1;
        float stepx = dx / nsteps;
        float stepy = dy / nsteps;

        for (int c = 0; c < nsteps; c++) {
            float _mx = lmx + c * stepx;
            float _my = lmy + c * stepy;

            int i = (int)((_mx / (float)1000) * N + 1);
            int j = (int)(((1000 - _my) / (float)1000) * N + 1);
            if (i < 1 || i > N || j < 1 || j > N) continue;

            float strength = ((float)(i + 1)) / nsteps;

            u_prev[CalcIndex(i, j)] = force * strength * ndx;
            v_prev[CalcIndex(i, j)] = force * strength * ndy;
            dens_prev[CalcIndex(i, j)] = source;
        }

        lmx = mx;
        lmy = my;
    }

    #endregion Funcs

    #region Solver

    void CalcDensity() {
        AddSource(ref dens, dens_prev);
        Swap(ref dens_prev, ref dens);
        Diffuse(0, ref dens, dens_prev);
        Swap(ref dens_prev, ref dens);
        Advect(0, ref dens, dens_prev, u, v);
    }

    void CalcVelocity() {
        AddSource(ref u, u_prev);
        AddSource(ref v, v_prev);

        Swap(ref u_prev, ref u);
        Diffuse(1, ref u, u_prev);

        Swap(ref v_prev, ref v);
        Diffuse(2, ref v, v_prev);
        Project(ref u, ref v, ref u_prev, ref v_prev);

        Swap(ref u_prev, ref u);
        Swap(ref v_prev, ref v);

        Advect(1, ref u, u_prev, u_prev, v_prev);
        Advect(2, ref v, v_prev, u_prev, v_prev);

        Project(ref u, ref v, ref u_prev, ref v_prev);
    }

    void AddSource(ref float[] x, float[] s) {
        for (int i = 0; i < bufferSize; ++i) {
            x[i] += dt * s[i];
        }
    }

    void SetBoundary(int b, ref float[] x) {
        for (int i = 1; i <= N; ++i) {
            x[CalcIndex(0, i)] = b == 1 ? -x[CalcIndex(1, i)] : x[CalcIndex(1, i)];
            x[CalcIndex(N + 1, i)] = b == 1 ? -x[CalcIndex(N, i)] : x[CalcIndex(N, i)];
            x[CalcIndex(i, 0)] = b == 2 ? -x[CalcIndex(i, 1)] : x[CalcIndex(i, 1)];
            x[CalcIndex(i, N + 1)] = b == 2 ? -x[CalcIndex(i, N)] : x[CalcIndex(i, N)];
        }
        x[CalcIndex(0, 0)] = 0.5f * (x[CalcIndex(1, 0)] + x[CalcIndex(0, 1)]);
        x[CalcIndex(0, N + 1)] = 0.5f * (x[CalcIndex(1, N + 1)] + x[CalcIndex(0, N)]);
        x[CalcIndex(N + 1, 0)] = 0.5f * (x[CalcIndex(N, 0)] + x[CalcIndex(N + 1, 1)]);
        x[CalcIndex(N + 1, N + 1)] = 0.5f * (x[CalcIndex(N, N + 1)] + x[CalcIndex(N + 1, N)]);
    }

    void Laplace(int b, ref float[] x, float[] x0, float a, float c) {
        for (int k = 0; k < 20; ++k) {
            for (int i = 1; i <= N; ++i) {
                for (int j = 1; j <= N; ++j) {
                    x[CalcIndex(i, j)] = decay * ( (x0[CalcIndex(i, j)] + a * (x[CalcIndex(i - 1, j)] + x[CalcIndex(i + 1, j)] + x[CalcIndex(i, j - 1)] + x[CalcIndex(i, j + 1)])) / c);
                }
            }
    
            SetBoundary(b, ref x);
        }
    }

    void Diffuse(int b, ref float[] x, float[] x0) {
        float a = dt * visc * N * N;
        Laplace(b, ref x, x0, a, 1 + 4 * a);
    }

    void Advect(int b, ref float[] d, float[] d0, float[] u, float[] v) {
        int i0, j0, i1, j1;
        float x, y, s0, t0, s1, t1, dt0;

        dt0 = dt * N;
        for (int i = 1; i <= N; ++i) {
            for (int j = 1; j <= N; ++j) {
                x = i - dt0 * u[CalcIndex(i, j)]; y = j - dt0 * v[CalcIndex(i, j)];
                if (x < 0.5f) x = 0.5f; if (x > N + 0.5f) x = N + 0.5f; i0 = (int)x; i1 = i0 + 1;
                if (y < 0.5f) y = 0.5f; if (y > N + 0.5f) y = N + 0.5f; j0 = (int)y; j1 = j0 + 1;
                s1 = x - i0; s0 = 1 - s1; t1 = y - j0; t0 = 1 - t1;
                d[CalcIndex(i, j)] = s0 * (t0 * d0[CalcIndex(i0, j0)] + t1 * d0[CalcIndex(i0, j1)]) + s1 * (t0 * d0[CalcIndex(i1, j0)] + t1 * d0[CalcIndex(i1, j1)]);
            }
        }

        SetBoundary(b, ref d);
    }

    void Project(ref float[] u, ref float[] v, ref float[] p, ref float[] div) {
        for (int i = 1; i <= N; ++i) {
            for (int j = 1; j <= N; ++j) {
                div[CalcIndex(i, j)] = -0.5f * (u[CalcIndex(i + 1, j)] - u[CalcIndex(i - 1, j)] + v[CalcIndex(i, j + 1)] - v[CalcIndex(i, j - 1)]) / N;
                p[CalcIndex(i, j)] = 0;
            }
        }

        SetBoundary(0, ref div);
        SetBoundary(0, ref p);

        Laplace(0, ref p, div, 1, 4);

        for (int i = 1; i <= N; ++i) {
            for (int j = 1; j <= N; ++j) {
                u[CalcIndex(i, j)] -= 0.5f * N * (p[CalcIndex(i + 1, j)] - p[CalcIndex(i - 1, j)]);
                v[CalcIndex(i, j)] -= 0.5f * N * (p[CalcIndex(i, j + 1)] - p[CalcIndex(i, j - 1)]);
            }
        }

        SetBoundary(1, ref u);
        SetBoundary(2, ref v);
    }

    void Swap(ref float[] src, ref float[] dst) {
        float[] tmp = src;
        src = dst;
        dst = tmp;
    }

    #endregion Solver
}
