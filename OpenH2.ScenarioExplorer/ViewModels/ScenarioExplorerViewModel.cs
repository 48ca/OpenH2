﻿using System.Collections.Generic;
using System.Collections.ObjectModel;
using PropertyChanged;
using System.ComponentModel;
using System.Linq;
using Avalonia.Controls;
using System;

namespace OpenH2.ScenarioExplorer.ViewModels
{
    [AddINotifyPropertyChangedInterface]
    public class ScenarioExplorerViewModel
    {
        public ScenarioExplorerViewModel()
        {
            
        }

        public ScenarioViewModel LoadedScenario { get; set; }

        private TagTreeEntryViewModel selectedEntry;
        public TagTreeEntryViewModel SelectedEntry
        {
            get => selectedEntry;
            set 
            {
                selectedEntry = value;

                if (selectedEntry != null)
                {
                    CurrentTag = LoadedScenario.GetTagViewModel(selectedEntry.Id);
                }
            }
        }

        private int selectedOffset;
        [DoNotCheckEquality]
        public int SelectedOffset
        {
            get => selectedOffset;
            set
            {
                selectedOffset = value;

                this.SelectedOffsetData = new DataPreviewViewModel(value, CurrentTag);
            }
        }

        private TagViewModel _currentTag;
        public TagViewModel CurrentTag
        {
            get => _currentTag;
            set
            {
                _currentTag = value;
                this.SelectedOffset = 0;
            }
        }

        public ObservableCollection<string> RecentFiles { get; set; } = new ObservableCollection<string>
        {
            "D:\\H2vMaps\\shared.map",
            "D:\\H2vMaps\\zanzibar.map",
            "D:\\H2vMaps\\ascension.map"
        };

        public Control[] MenuItems { get; set; }

        public DataPreviewViewModel SelectedOffsetData { get; set; }

        public bool DisableHexViewer { get; set; }
    }
}
