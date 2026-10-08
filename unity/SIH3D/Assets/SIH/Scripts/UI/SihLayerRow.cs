// One row of the layers panel: click to toggle the layer (same as its key).

using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

namespace Sih
{
    public class SihLayerRow : MonoBehaviour, IPointerClickHandler
    {
        public SihReplay replay;
        [Range(0, LayerSet.Count - 1)] public int layer;
        public Image keyBox;
        public Text keyText;
        public Text nameText;

        [Header("Colours")]
        public Color keyOn = Palette.A(Palette.Plan, 0.9f);
        public Color keyOff = Palette.Line;
        public Color nameOn = Palette.Ink;
        public Color nameOff = Palette.A(Palette.Dim, 0.6f);

        public void Show(bool on)
        {
            if (keyBox != null && keyBox.color != (on ? keyOn : keyOff)) keyBox.color = on ? keyOn : keyOff;
            if (nameText != null && nameText.color != (on ? nameOn : nameOff)) nameText.color = on ? nameOn : nameOff;
        }

        public void OnPointerClick(PointerEventData e)
        {
            if (replay != null) replay.ToggleLayer(layer);
        }

#if UNITY_EDITOR
        void OnValidate()
        {
            if (!Application.isPlaying) SihReplay.RequestEditorRefresh();
        }
#endif
    }
}
