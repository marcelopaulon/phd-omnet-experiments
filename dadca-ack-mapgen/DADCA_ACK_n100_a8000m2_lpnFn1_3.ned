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

network DADCA_ACK_n100_a8000m2_lpnFn1_3
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
        @display("i=misc/sensor,p=95.42,179.64");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=140.44,227.73");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=185.65,295.93");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=96.87,276.24");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=33.77,268.95");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=217.38,178.50");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=252.09,300.33");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=299.31,357.82");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=263.69,232.63");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=187.57,116.51");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=273.36,113.77");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=33.65,65.33");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=11.28,143.39");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=206.53,49.98");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=269.23,52.90");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=332.02,34.58");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=370.11,102.97");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=460.46,126.33");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=511.73,167.87");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=207.45,403.54");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=128.91,453.13");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=69.72,494.85");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=124.60,354.29");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=34.15,374.63");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=138.48,13.83");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=3.87,10.63");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=629.34,151.76");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=578.30,234.85");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=736.47,130.74");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=809.11,179.98");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=704.87,50.60");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=646.05,21.78");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=641.60,82.38");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=773.42,50.42");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=848.01,52.77");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=695.65,203.30");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=895.34,6.87");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=937.20,85.80");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=874.22,214.13");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=838.52,293.06");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=883.56,338.34");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=859.46,405.26");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=823.76,533.03");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=879.68,472.47");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=941.04,487.55");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=979.08,434.86");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=770.46,367.72");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=769.56,304.19");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=758.61,213.39");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=634.58,258.69");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=619.94,343.95");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=526.68,373.45");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=562.47,322.41");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=504.25,256.33");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=432.36,305.82");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=382.73,238.67");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=420.39,34.02");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=498.91,60.02");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=576.47,60.06");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=1018.40,145.74");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=1099.28,104.03");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=1131.72,155.44");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=1100.02,215.63");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=999.26,62.42");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=867.77,131.49");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=1032.66,239.10");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=972.75,283.60");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=1103.74,310.40");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1106.89,429.46");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1041.91,492.55");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=993.08,532.77");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=1060.64,576.01");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=878.01,564.48");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=959.41,636.45");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=1106.18,715.35");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=1166.22,767.42");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=1043.56,701.85");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=957.45,721.45");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=894.84,660.50");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=863.89,739.17");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=879.45,835.39");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=936.53,799.15");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=1033.65,784.36");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=1132.21,871.08");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1188.47,909.24");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=1280.27,865.44");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=1200.85,653.61");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=1264.78,626.89");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=1145.21,504.10");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=1219.15,520.46");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1134.09,563.73");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1304.06,682.78");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1331.63,590.77");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1254.47,462.09");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1283.04,545.06");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1183.63,399.11");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1234.12,286.51");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1208.06,215.49");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1259.04,141.24");
    }

}

