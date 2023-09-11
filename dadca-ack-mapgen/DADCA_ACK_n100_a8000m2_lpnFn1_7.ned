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

network DADCA_ACK_n100_a8000m2_lpnFn1_7
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
        @display("i=misc/sensor,p=198.33,115.71");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=266.08,57.74");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=319.57,101.25");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=406.47,109.77");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=389.24,33.61");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=410.45,212.39");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=317.90,164.72");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=269.46,239.66");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=213.91,267.00");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=158.77,210.12");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=73.71,247.52");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=128.81,302.59");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=101.51,358.30");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=17.83,371.51");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=53.09,441.09");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=142.82,476.19");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=172.41,397.47");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=238.32,404.82");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=281.08,491.90");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=346.78,433.91");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=414.93,372.00");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=311.44,289.55");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=395.15,299.30");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=103.98,166.10");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=0.06,146.22");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=220.41,183.29");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=500.40,238.67");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=491.06,313.00");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=473.02,444.98");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=553.18,472.14");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=491.75,535.83");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=408.26,528.66");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=438.47,610.61");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=340.82,529.43");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=567.86,559.55");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=631.05,517.68");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=653.67,613.34");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=625.27,405.25");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=708.31,376.81");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=763.31,350.74");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=765.15,271.16");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=756.99,196.29");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=789.22,93.15");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=732.94,130.29");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=690.57,19.00");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=607.18,94.83");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=542.87,96.17");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=526.51,20.54");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=462.77,39.06");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=444.48,159.82");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=547.75,167.87");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=532.19,610.06");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=577.94,650.53");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=556.77,714.45");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=617.24,707.99");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=645.60,799.49");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=719.96,800.23");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=524.52,395.99");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=633.07,205.67");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=667.33,144.62");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=612.82,23.78");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=676.32,79.77");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=830.00,22.03");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=891.44,27.87");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=880.35,145.74");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=967.37,167.02");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=1020.45,82.66");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=896.15,258.57");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=832.15,218.08");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=843.58,348.57");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=887.72,430.63");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=818.73,420.52");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=745.94,451.65");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=684.10,465.36");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=573.49,342.31");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=735.47,610.18");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=694.63,535.93");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=453.86,690.83");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=400.93,758.66");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=348.09,710.25");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=376.35,651.08");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=447.36,822.28");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=470.72,757.48");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=518.73,811.66");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=347.85,825.04");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=297.57,882.97");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=236.67,832.64");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=219.13,763.24");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=123.38,751.54");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=106.49,849.02");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=33.62,909.55");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=43.12,832.75");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=54.95,749.13");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=208.87,652.64");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=304.09,623.13");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=221.40,547.11");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=123.94,567.47");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=46.13,529.35");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=276.47,724.70");
    }

}

