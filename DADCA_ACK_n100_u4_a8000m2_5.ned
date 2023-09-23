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

network DADCA_ACK_n100_u4_a8000m2_5
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
        @display("i=misc/sensor;p=240.77,7988.04");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=853.54,7878.55");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1368.01,7980.16");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3005.09,7856.43");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3521.47,7757.57");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4399.13,7977.83");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1309.18,7582.31");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1888.99,7674.60");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2486.14,7723.70");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4985.23,7704.06");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=654.76,7504.89");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2126.89,7362.77");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2705.62,7258.20");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3247.59,7356.92");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4008.02,7414.42");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4652.41,7395.65");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5235.52,7276.83");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=74.73,7194.01");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=407.67,7018.88");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=861.54,7075.21");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1182.88,7250.60");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1528.19,7064.37");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3673.77,7078.11");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5772.96,7034.13");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2300.05,6796.99");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2647.83,6905.76");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3302.13,6803.67");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4532.75,6775.07");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4935.53,6808.89");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1664.38,6600.51");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3974.37,6667.97");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5468.48,6741.04");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5972.61,6694.96");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=336.65,6364.92");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=989.93,6413.79");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2145.37,6391.75");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2603.95,6476.31");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3201.53,6306.55");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3574.31,6302.23");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4325.87,6382.21");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4683.60,6312.31");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2032.55,6032.31");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2508.22,6089.88");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3988.50,6159.70");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5076.71,6131.44");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5656.82,6138.02");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=85.49,5916.69");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=715.86,5978.65");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1536.93,5939.71");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3153.29,5805.54");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3903.83,5787.00");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4516.53,5902.62");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4905.69,5783.16");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5965.53,5794.80");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1042.42,5740.23");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1618.94,5567.23");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2575.35,5642.98");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3555.14,5736.54");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4334.40,5552.08");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5553.07,5600.03");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=658.22,5386.04");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1073.06,5286.15");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2217.69,5508.70");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3108.36,5278.70");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3570.03,5277.81");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3969.72,5371.31");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4915.64,5296.54");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=166.04,5261.35");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2539.31,5190.65");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5379.75,5226.05");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=547.49,4885.98");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1023.83,4857.89");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1686.27,4985.56");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3052.46,4812.03");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4149.29,4885.73");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4541.96,4912.30");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5174.79,4819.72");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5742.19,4942.53");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=17.14,4653.04");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1521.90,4665.03");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2695.32,4642.79");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3805.49,4780.08");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5831.95,4551.71");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=479.90,4526.37");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1247.28,4287.98");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2093.31,4446.61");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3240.50,4434.57");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3619.74,4434.51");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4275.21,4452.27");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4658.80,4491.45");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4995.57,4304.80");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5307.77,4466.16");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=490.15,4155.74");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=859.82,4127.06");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1785.95,4134.74");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2724.69,4206.51");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3089.77,4083.64");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4394.92,4066.61");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5629.15,4148.28");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2205.59,3926.39");
    }

}

