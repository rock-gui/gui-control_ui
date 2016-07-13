
Vizkit::UiLoader::extend_cplusplus_widget_class "ControlUi" do

    #called when the widget is created
    def initialize_vizkit_extension
        #activate Typelib transport via qt slots
        extend Vizkit::QtTypelibExtension
        setContextMenuPolicy(Qt::CustomContextMenu)
        connect( SIGNAL('customContextMenuRequested(const QPoint)')) do |pos|
            fileMenu = Qt::Menu.new(Qt::MainWindow.tr("File"))
            importAction = fileMenu.addAction(Qt::MainWindow.tr("Import Joint Values"))
            importAction.connect(SIGNAL('triggered()')) do
                dialog = Qt::FileDialog.new()
                dialog.setFileMode(Qt::FileDialog::AnyFile)
                dialog.setNameFilter("YAML configuration files (*.yml)")
                dialog.setDirectory(Qt::Dir::homePath())
                if (dialog.exec())
                    fileNames = dialog.selectedFiles()
                    if fileNames.size > 0 and not File.directory?(fileNames.first)
                        checkUpdateCB(false)
                        checkKeepSendingCB(false) 
                        f = open(fileNames.first,'r')
                        yaml = YAML.load f.read
                        f.close
                        type_instance = Types::Base::Samples::Joints.new
                        Orocos::TaskConfigurations.typelib_from_yaml_value(
                                type_instance, yaml)
                        setReference(type_instance)  
                    end
                end   
            end
            exportAction = fileMenu.addAction(Qt::MainWindow.tr("Export Joint Values"))
            exportAction.connect(SIGNAL('triggered()')) do
                j=getJoints()
                yaml = Orocos::TaskConfigurations.typelib_to_yaml_value(j).to_yaml
                puts yaml
                dialog = Qt::FileDialog.new()
                dialog.setFileMode(Qt::FileDialog::AnyFile)
                dialog.setNameFilter("YAML configuration files (*.yml)")
                dialog.setDirectory(Qt::Dir::homePath())
                if (dialog.exec())
                    fileNames = dialog.selectedFiles()
                    if fileNames.size > 0 and not File.directory?(fileNames.first)
                        f = open(fileNames.first,'w')
                        f.write yaml
                        f.close
                    end
                end
            end
            fileMenu.exec( mapToGlobal(pos))
        end
    end

    #called each time vizkit wants to display a new 
    #port with this widget
    def config(value,options)

    end

    #called each time new data are available on the 
    #orocos port connected to the widget the name is
    #custom and can be set via register_widget_for
    def update(sample,port_name)
        #mySlot(sample)
    end
end

# register widget for a specific Typelib type to be 
# accessible via rock tooling (rock-replay,...)
# multiple register_widget_for are allowed for each widget
# Vizkit::UiLoader.register_widget_for("ControlUi","/base/Angle",:update)
