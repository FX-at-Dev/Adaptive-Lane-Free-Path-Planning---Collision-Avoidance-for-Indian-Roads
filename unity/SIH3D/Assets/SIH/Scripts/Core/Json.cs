// Minimal JSON reader for the run files written by sim/sih_export_run.m.
//
// The project carries its own reader rather than a package so a build has no
// registry dependency and runs on a machine with no network, which is the
// demo laptop on stage. It handles exactly JSON: objects, arrays, strings,
// numbers, true/false/null. Numbers are read as double.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace Sih
{
    public static class Json
    {
        public static object Parse(string s)
        {
            int i = 0;
            object v = Value(s, ref i);
            return v;
        }

        static void Ws(string s, ref int i)
        {
            while (i < s.Length && (s[i] == ' ' || s[i] == '\n' || s[i] == '\r' || s[i] == '\t')) i++;
        }

        static object Value(string s, ref int i)
        {
            Ws(s, ref i);
            char c = s[i];
            if (c == '{') return Obj(s, ref i);
            if (c == '[') return Arr(s, ref i);
            if (c == '"') return Str(s, ref i);
            if (c == 't') { i += 4; return true; }
            if (c == 'f') { i += 5; return false; }
            if (c == 'n') { i += 4; return null; }
            return Num(s, ref i);
        }

        static Dictionary<string, object> Obj(string s, ref int i)
        {
            var d = new Dictionary<string, object>();
            i++;
            Ws(s, ref i);
            if (s[i] == '}') { i++; return d; }
            while (true)
            {
                Ws(s, ref i);
                string k = Str(s, ref i);
                Ws(s, ref i);
                i++; // ':'
                d[k] = Value(s, ref i);
                Ws(s, ref i);
                if (s[i] == ',') { i++; continue; }
                i++; // '}'
                return d;
            }
        }

        static List<object> Arr(string s, ref int i)
        {
            var a = new List<object>();
            i++;
            Ws(s, ref i);
            if (s[i] == ']') { i++; return a; }
            while (true)
            {
                a.Add(Value(s, ref i));
                Ws(s, ref i);
                if (s[i] == ',') { i++; continue; }
                i++; // ']'
                return a;
            }
        }

        static string Str(string s, ref int i)
        {
            i++; // opening quote
            var sb = new StringBuilder();
            while (s[i] != '"')
            {
                char c = s[i++];
                if (c != '\\') { sb.Append(c); continue; }
                char e = s[i++];
                switch (e)
                {
                    case 'n': sb.Append('\n'); break;
                    case 't': sb.Append('\t'); break;
                    case 'r': sb.Append('\r'); break;
                    case 'b': sb.Append('\b'); break;
                    case 'f': sb.Append('\f'); break;
                    case 'u':
                        sb.Append((char)Convert.ToInt32(s.Substring(i, 4), 16));
                        i += 4;
                        break;
                    default: sb.Append(e); break;
                }
            }
            i++;
            return sb.ToString();
        }

        static double Num(string s, ref int i)
        {
            int j = i;
            while (j < s.Length && "+-0123456789.eE".IndexOf(s[j]) >= 0) j++;
            double v = double.Parse(s.Substring(i, j - i), NumberStyles.Float, CultureInfo.InvariantCulture);
            i = j;
            return v;
        }

        // ---- typed access ------------------------------------------------------
        // Octave's jsonencode collapses a 1xN matrix to a flat list, a scalar to
        // a bare number and an empty matrix to []. These helpers accept every
        // one of those shapes, so the readers above them never special-case it.

        public static Dictionary<string, object> O(object o, string k)
        {
            return (o as Dictionary<string, object>)?[k] as Dictionary<string, object>;
        }

        public static bool Has(object o, string k)
        {
            var d = o as Dictionary<string, object>;
            return d != null && d.ContainsKey(k) && d[k] != null;
        }

        public static object Get(object o, string k)
        {
            var d = o as Dictionary<string, object>;
            return d != null && d.TryGetValue(k, out var v) ? v : null;
        }

        public static double D(object o, string k, double dflt = 0)
        {
            var v = Get(o, k);
            if (v is double x) return x;
            if (v is bool b) return b ? 1 : 0;
            if (v is List<object> l && l.Count > 0 && l[0] is double y) return y;
            return dflt;
        }

        public static string S(object o, string k, string dflt = "")
        {
            return Get(o, k) as string ?? dflt;
        }

        /// Flattened numeric array, whatever nesting the encoder produced.
        public static float[] F(object v)
        {
            var outp = new List<float>();
            Flatten(v, outp);
            return outp.ToArray();
        }

        public static float[] F(object o, string k) => F(Get(o, k));

        static void Flatten(object v, List<float> outp)
        {
            if (v is double d) { outp.Add((float)d); return; }
            if (v is List<object> l) foreach (var e in l) Flatten(e, outp);
        }

        /// List of strings; a single string arrives bare and is wrapped.
        public static string[] Strs(object v)
        {
            if (v is string s) return new[] { s };
            var l = v as List<object>;
            if (l == null) return new string[0];
            var r = new string[l.Count];
            for (int i = 0; i < l.Count; i++) r[i] = l[i] as string ?? "";
            return r;
        }

        /// Rows of a matrix. A 2xN exported matrix arrives as [[...],[...]];
        /// a 1xN as a flat list, which is returned as a single row.
        public static List<float[]> Rows(object v)
        {
            var r = new List<float[]>();
            var l = v as List<object>;
            if (l == null || l.Count == 0) return r;
            if (l[0] is double) { r.Add(F(l)); return r; }
            foreach (var e in l) r.Add(F(e));
            return r;
        }
    }
}
