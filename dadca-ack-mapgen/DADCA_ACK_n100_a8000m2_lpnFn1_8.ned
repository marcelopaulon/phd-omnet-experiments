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
        @display("i=misc/sensor,p=142.53,184.62");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=241.58,183.39");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=250.62,117.54");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=261.13,26.51");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=361.14,64.40");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=356.20,181.25");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=60.15,236.25");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=117.05,313.28");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=123.44,412.13");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=214.91,391.29");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=184.14,313.66");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=247.50,334.78");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=328.37,259.95");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=414.45,293.38");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=350.50,351.96");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=333.64,442.91");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=372.25,511.97");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=344.52,582.79");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=306.81,647.76");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=375.25,706.51");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=238.36,762.28");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=168.35,741.79");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=95.98,777.01");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=34.67,722.48");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=110.48,683.25");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=16.09,639.27");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=88.24,620.93");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=154.91,576.64");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=183.02,492.00");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=239.14,547.70");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=452.25,552.14");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=461.40,624.40");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=513.33,662.23");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=577.83,525.59");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=539.03,602.29");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=660.02,495.07");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=555.55,454.73");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=542.58,373.69");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=601.88,410.88");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=696.85,423.34");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=740.99,482.39");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=796.77,410.39");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=863.11,476.26");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=888.46,554.47");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=951.96,586.55");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1039.27,553.21");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1022.90,626.34");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=1060.44,676.89");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=926.04,644.97");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=817.76,671.15");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=747.04,548.82");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=632.62,319.66");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=694.08,269.76");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=774.13,216.67");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=689.49,182.85");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=516.33,182.29");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=600.55,223.56");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=458.51,365.70");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=498.10,243.68");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=802.82,342.60");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=749.28,371.02");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=844.27,238.78");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=895.42,297.63");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=941.33,375.47");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=626.51,141.63");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=556.49,62.11");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=775.56,124.70");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=978.45,312.71");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1042.18,286.18");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1091.43,349.47");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=1149.87,389.82");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=1208.86,447.36");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=1160.64,503.67");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=1069.96,494.14");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=1090.44,622.45");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=1166.33,599.77");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=1266.54,589.80");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=1221.22,639.36");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=1193.72,709.39");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=1212.55,779.71");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=1151.80,774.95");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=1139.90,846.46");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=1060.59,888.19");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=989.50,821.68");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1049.47,770.40");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=964.76,746.00");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=966.93,885.21");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=857.02,834.04");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=795.22,893.50");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=740.33,846.21");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=668.96,824.62");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=903.81,872.39");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=997.22,1003.14");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=907.17,1030.99");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=964.21,1063.97");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1026.86,1080.33");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1059.27,978.45");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1113.34,1052.84");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1131.12,1167.40");
    }

}

