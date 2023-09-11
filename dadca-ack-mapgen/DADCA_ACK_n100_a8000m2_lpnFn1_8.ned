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

network DADCA_ACK_n100_a8000m2_lpnFn1_8
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
        @display("i=misc/sensor,p=106.76,2.95");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=201.55,48.87");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=207.85,135.53");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=198.98,230.43");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=287.85,96.96");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=296.37,177.72");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=278.92,313.10");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=218.21,387.07");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=127.54,338.89");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=124.30,264.09");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=193.82,300.22");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=289.80,404.42");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=339.50,368.25");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=305.99,247.69");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=127.27,184.62");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=355.31,210.87");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=421.04,312.05");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=377.21,268.16");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=414.19,71.36");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=447.32,205.98");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=496.10,107.21");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=550.27,165.53");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=506.95,221.80");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=543.06,349.87");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=423.36,386.38");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=636.01,175.39");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=699.51,148.55");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=560.38,279.16");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=620.80,305.53");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=680.53,366.87");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=581.40,422.32");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=520.49,416.41");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=428.34,452.89");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=465.36,520.30");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=547.01,528.56");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=438.71,598.96");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=692.34,301.28");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=646.20,240.69");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=336.59,20.75");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=350.08,145.51");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=24.82,47.34");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=30.22,145.55");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=37.86,236.19");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=32.48,297.93");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=123.20,435.40");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=239.33,524.83");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=275.03,592.34");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=208.38,601.19");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=141.96,597.15");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=125.66,508.03");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=62.28,554.79");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=55.19,623.52");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=59.21,701.85");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=120.66,657.60");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=203.16,680.32");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=272.14,763.48");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=209.53,741.15");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=274.24,675.02");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=330.79,528.62");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=357.69,458.53");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=609.46,573.82");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=562.30,618.45");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=650.69,639.60");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=576.95,725.51");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=656.75,775.29");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=707.21,813.49");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=690.03,902.31");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=614.05,859.37");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=547.18,909.32");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=628.74,946.03");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=802.59,762.86");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=727.89,753.23");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=608.32,1036.53");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=687.11,1072.47");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=755.85,1093.93");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=771.51,901.39");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=851.39,903.16");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=839.41,1009.23");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=828.12,1097.23");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=787.92,1157.86");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=645.20,1147.39");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=724.65,1024.30");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=775.76,987.53");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=582.35,1135.09");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=620.27,1225.52");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=652.37,1294.71");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=578.93,1329.05");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=493.97,1298.07");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=426.64,1350.56");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=460.93,1427.70");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=523.06,1353.41");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=531.57,1246.42");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=485.87,1146.02");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=392.36,1121.66");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=340.78,1168.37");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=327.13,1239.58");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=365.09,1305.48");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=258.61,1292.20");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=299.51,1371.88");
    }

}

