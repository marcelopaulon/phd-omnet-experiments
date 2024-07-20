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

network DADCA_ACK_n100_a8000m2_lpnFn1_0
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
        @display("i=misc/sensor,p=33.78,118.66");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=46.82,179.52");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=113.00,181.61");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=183.09,129.58");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=120.23,271.93");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=191.37,254.87");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=111.69,340.27");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=44.37,270.65");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=43.86,358.91");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=186.50,355.59");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=133.67,413.24");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=176.50,463.47");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=238.75,421.47");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=261.29,490.96");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=298.96,575.59");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=307.85,362.33");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=335.01,481.22");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=247.92,291.23");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=221.29,182.52");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=41.60,29.44");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=105.29,16.62");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=174.02,17.98");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=226.91,62.32");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=283.27,102.71");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=319.50,23.81");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=262.30,0.64");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=111.40,484.71");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=159.21,521.31");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=127.82,587.50");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=414.60,416.61");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=329.21,232.02");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=405.54,223.87");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=389.44,287.88");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=491.39,334.49");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=354.39,123.92");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=435.05,42.32");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=521.12,90.54");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=500.53,181.68");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=575.67,184.58");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=657.09,137.49");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=726.41,139.85");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=817.71,142.04");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=722.74,211.16");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=698.20,307.06");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=798.43,336.97");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=818.92,228.74");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=887.19,268.78");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=956.86,265.14");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=862.84,383.85");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=788.77,447.16");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=729.81,468.17");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=674.77,403.88");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=663.59,485.38");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=678.80,573.07");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=741.90,603.76");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=749.54,541.68");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=721.73,669.13");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=800.94,692.17");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=885.84,676.37");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=962.07,655.24");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=1011.63,706.51");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=1044.13,640.53");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=953.16,782.79");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=853.59,774.27");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=760.32,775.66");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=781.85,843.42");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=644.65,664.04");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=621.41,728.01");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=545.42,681.01");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=496.63,767.12");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=442.84,706.20");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=378.94,654.87");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=459.05,613.18");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=594.89,559.11");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=586.93,635.19");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=570.09,468.38");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=550.99,410.62");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=474.35,465.86");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=402.47,348.91");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=49.10,658.47");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=42.27,578.82");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=132.86,698.14");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=69.59,735.08");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=224.16,571.51");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=247.84,704.02");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=314.77,756.79");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=237.67,764.53");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=287.42,891.01");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=215.93,874.98");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=130.73,792.36");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=169.44,645.38");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=10.26,522.62");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=14.01,783.61");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=62.36,836.99");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=50.10,922.27");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=132.18,963.43");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=178.83,924.41");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=314.17,1021.85");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=395.29,1026.84");
    }

}

