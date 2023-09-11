package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n100_a8000m2_lpnFn1_6
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=136.39,192.59");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=127.32,283.19");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=76.36,337.26");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=81.06,416.30");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=46.14,470.33");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=155.08,442.18");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=220.41,464.89");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=153.91,558.87");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=116.94,500.16");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=206.71,529.09");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=24.72,596.30");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=175.31,337.27");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=217.06,400.68");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=280.13,538.81");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=241.30,639.43");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=239.48,724.52");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=248.42,818.87");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=241.09,895.46");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=276.62,982.63");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=241.31,1048.80");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=334.77,1077.33");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=176.98,731.57");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=151.76,671.64");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=305.48,600.56");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=347.37,511.13");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=331.65,436.34");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=405.87,539.02");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=502.75,533.99");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=408.55,353.36");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=482.59,290.42");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=526.92,343.81");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=564.19,251.70");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=596.63,312.92");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=582.91,192.38");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=641.84,167.88");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=657.09,87.71");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=590.76,84.55");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=505.36,109.18");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=713.84,185.34");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=680.03,273.95");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=763.75,323.71");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=855.08,336.05");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=931.20,282.14");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=936.83,369.35");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=927.92,455.51");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=840.42,418.51");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=689.53,355.98");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=646.88,443.34");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=592.37,524.55");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=552.10,605.40");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=477.09,591.03");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=480.69,667.24");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=517.36,729.93");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=449.07,787.65");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=513.56,845.56");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=575.11,753.88");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=656.83,803.18");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=669.62,874.87");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=753.28,916.56");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=746.04,977.86");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=753.73,1065.89");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=659.47,1096.88");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=580.21,1096.75");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=539.15,1013.42");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=611.94,989.53");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=518.22,928.40");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=464.30,1057.09");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=733.91,850.60");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=801.08,824.78");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=882.61,817.98");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=951.61,845.90");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=987.78,793.52");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=1051.10,766.41");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=970.10,732.00");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=899.79,753.89");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=1126.33,843.19");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=1156.42,712.68");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=833.25,889.63");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=729.35,728.19");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=655.52,677.65");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=673.54,610.99");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=647.63,551.72");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=783.93,604.75");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=841.20,678.02");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=893.60,637.00");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=885.65,576.11");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=829.88,548.04");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=958.17,640.10");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=1011.46,525.02");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=1036.47,631.09");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1060.71,567.29");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1095.75,473.33");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1155.21,451.12");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1215.10,520.02");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1266.10,450.27");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1287.86,523.34");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1343.67,446.46");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1409.42,383.04");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1330.56,375.85");
    }

}

