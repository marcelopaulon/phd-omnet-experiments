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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=153.74,170.09");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=212.63,237.60");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=234.55,167.09");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=169.40,280.38");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=154.93,363.90");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=304.20,298.27");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=320.77,210.60");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=285.97,124.49");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=228.17,102.36");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=84.21,255.80");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=43.20,178.14");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=369.26,250.40");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=424.69,303.00");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=369.08,383.94");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=429.71,381.54");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=501.17,364.19");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=561.77,432.39");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=621.97,429.25");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=645.27,343.00");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=725.27,432.12");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=622.09,265.13");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=531.49,280.31");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=423.31,449.14");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=362.87,476.36");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=392.57,530.79");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=467.08,596.30");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=430.68,658.42");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=370.02,694.16");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=305.34,691.02");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=212.04,660.05");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=227.56,743.36");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=288.93,776.54");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=375.23,776.75");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=236.25,889.58");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=171.93,922.67");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=86.36,896.76");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=156.08,862.82");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=210.88,1016.54");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=211.14,1116.42");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=239.83,1211.33");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=296.41,1278.55");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=270.25,1333.19");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=268.87,1402.01");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=202.54,1373.19");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=132.45,1344.27");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=123.53,1280.15");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=108.20,1212.27");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=213.03,1304.65");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=145.65,1130.05");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=169.19,1059.73");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=225.82,951.05");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=355.25,915.58");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=311.07,970.37");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=284.75,1035.91");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=453.51,1004.17");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=523.28,956.98");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=516.42,1018.70");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=548.95,1089.72");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=576.84,1173.37");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=513.27,1171.96");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=614.18,1040.03");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=689.54,1011.80");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=733.18,956.30");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=676.20,905.45");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=589.99,863.90");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=466.46,827.04");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=477.78,759.61");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=515.62,635.52");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=599.92,613.07");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=584.82,690.75");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=658.74,555.10");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=584.38,517.62");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=525.32,549.06");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=465.36,534.60");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=734.66,556.16");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=734.29,630.48");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=801.30,564.63");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=902.69,506.57");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=944.40,429.98");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=906.47,379.72");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=903.64,293.23");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=871.37,567.70");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=807.12,504.76");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=843.60,623.56");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=836.40,711.15");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=917.82,689.06");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1003.80,701.86");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=994.46,634.59");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1086.86,649.25");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1086.46,561.25");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1110.73,497.66");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1033.36,489.11");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1002.14,346.82");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1003.34,253.18");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1064.37,278.01");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1074.25,218.80");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=916.97,211.35");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=954.14,121.40");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=832.59,78.85");
    }

}

