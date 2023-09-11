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
        @display("i=misc/sensor,p=111.50,181.79");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=22.58,141.49");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=22.25,71.96");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=154.51,125.57");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=227.49,82.30");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=183.69,8.73");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=113.35,35.52");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=21.04,6.85");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=64.62,268.79");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=217.90,198.96");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=20.44,378.74");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=82.57,385.37");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=174.95,385.53");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=159.84,290.28");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=6.13,211.80");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=288.37,87.91");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=333.96,137.22");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=272.52,161.73");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=0.08,299.80");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=379.84,9.66");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=385.89,91.90");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=401.41,177.23");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=491.61,206.06");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=419.39,235.93");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=329.53,251.90");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=263.13,315.65");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=409.60,316.49");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=454.41,110.14");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=515.22,81.78");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=558.81,146.17");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=555.25,268.13");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=610.23,320.22");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=632.99,194.08");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=488.29,269.92");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=691.64,241.95");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=731.11,310.14");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=787.44,237.94");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=653.71,364.82");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=603.00,412.11");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=538.76,353.31");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=706.56,123.60");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=671.28,30.80");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=618.28,106.40");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=606.65,27.34");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=539.02,11.85");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=371.16,362.76");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=411.00,439.60");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=462.28,382.82");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=689.93,419.72");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=756.10,456.49");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=744.15,372.36");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=798.53,171.72");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=869.89,162.01");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=900.35,216.27");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=828.47,110.89");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=889.60,290.44");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=851.65,365.66");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=809.20,494.78");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=870.13,479.88");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=831.69,557.19");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=613.08,522.01");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=585.35,615.18");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=548.76,698.13");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=475.73,663.90");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=418.41,734.88");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=390.28,841.39");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=380.51,675.19");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=335.76,611.23");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=246.62,649.69");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=219.68,720.01");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=162.18,756.37");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=100.90,826.82");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=64.09,747.07");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=3.20,759.50");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=17.62,820.10");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=145.61,898.58");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=172.81,971.68");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=74.62,941.65");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=85.31,1012.01");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=14.90,1067.28");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=25.84,996.52");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=92.63,1113.68");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=160.38,1107.54");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=122.65,1173.84");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=87.98,1254.93");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=175.47,1208.73");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=249.36,1163.09");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=273.86,1230.88");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=318.40,1148.22");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=264.05,1096.03");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=320.58,1029.78");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=306.92,968.98");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=367.00,1189.13");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=442.38,1160.06");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=456.33,1241.90");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=453.86,1337.18");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=485.82,1406.62");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=399.49,1284.39");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=235.19,1004.22");
    }

}

