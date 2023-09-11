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

network DADCA_ACK_n100_a8000m2_lpnFn1_5
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
        @display("i=misc/sensor,p=166.27,143.19");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=144.89,5.03");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=149.32,65.66");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=219.19,81.29");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=224.05,163.05");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=300.34,159.41");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=274.05,229.04");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=325.98,293.39");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=383.20,249.80");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=442.27,205.06");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=525.60,156.89");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=542.92,237.02");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=565.67,319.26");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=625.51,111.99");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=687.63,143.96");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=696.72,227.55");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=642.39,287.27");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=473.24,283.15");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=595.62,193.62");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=439.84,49.31");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=387.79,154.75");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=345.21,107.41");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=380.49,37.79");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=204.93,258.06");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=142.73,269.48");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=68.16,306.52");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=88.26,228.26");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=20.62,261.72");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=154.84,363.20");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=90.87,373.31");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=102.67,472.56");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=48.65,576.78");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=111.98,553.06");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=150.53,612.40");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=208.69,650.80");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=267.50,691.24");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=302.68,752.35");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=384.70,689.54");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=421.65,785.70");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=367.96,847.53");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=396.44,950.17");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=302.20,956.99");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=339.33,904.17");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=268.92,847.35");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=202.25,887.89");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=148.73,916.57");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=103.07,833.28");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=94.87,1001.15");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=20.41,1007.52");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=46.09,951.59");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=38.41,873.39");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=237.58,773.48");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=458.00,722.12");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=475.68,654.98");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=414.92,591.85");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=337.19,596.20");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=223.85,570.21");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=155.05,704.25");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=52.06,690.49");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=29.22,473.00");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=3.08,386.83");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=218.21,337.28");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=25.51,120.49");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=684.02,73.22");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=702.47,314.00");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=683.96,380.29");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=652.51,466.08");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=716.95,483.06");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=574.37,400.97");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=586.77,478.71");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=511.61,420.80");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=467.08,343.09");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=830.97,410.16");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=788.16,325.21");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=554.73,575.35");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=507.34,525.84");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=441.33,399.84");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=468.95,468.47");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=628.50,583.82");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=530.73,685.13");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=576.43,633.41");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=685.24,558.38");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=699.53,643.28");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=777.53,603.25");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=333.12,378.63");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=258.15,412.18");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=317.86,447.76");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=377.80,454.04");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=291.09,554.77");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=424.13,519.62");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1.11,822.79");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=16.14,758.88");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=490.49,776.36");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=567.24,836.37");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=536.94,888.69");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=505.78,971.26");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=455.53,925.78");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=570.72,1004.70");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=496.66,1039.17");
    }

}

