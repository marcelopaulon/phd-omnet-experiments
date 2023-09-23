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

network DADCA_ACK_n100_u8_a8000m2_4
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
        @display("i=misc/sensor;p=356.85,7836.74");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=860.89,7802.94");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1230.86,7825.88");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1782.92,7875.65");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2558.26,7777.56");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3880.82,7779.01");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4995.12,7801.25");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5790.17,7942.32");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=76.61,7548.75");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2175.15,7719.86");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3107.69,7592.70");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4476.29,7720.96");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5344.10,7692.22");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=422.48,7477.20");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1063.93,7374.46");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1441.54,7381.12");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1878.83,7455.51");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2500.93,7424.36");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5996.98,7499.09");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=733.09,7126.73");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1434.88,7017.51");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2273.57,7092.90");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2626.62,7073.34");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3089.01,7238.61");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3600.51,7151.19");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4078.34,7248.09");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4527.85,7218.82");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4937.64,7202.32");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5298.17,7213.80");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5813.17,7181.71");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=175.10,6822.81");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1117.28,6812.79");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1918.99,6951.97");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3077.33,6806.48");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4124.10,6771.39");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=517.84,6530.66");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2208.86,6664.86");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2640.04,6595.74");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3477.94,6554.01");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5197.15,6686.71");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1746.74,6513.23");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2185.25,6273.54");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3851.14,6412.20");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4252.77,6268.87");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4506.66,6510.91");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=242.68,6198.55");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=858.97,6062.21");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1485.49,6129.96");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2632.79,6156.59");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4716.56,6140.24");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5070.43,6119.39");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5731.52,6247.96");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1731.68,5816.07");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2659.36,5773.63");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3275.73,5959.09");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3819.97,6015.45");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=13.01,5697.66");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=678.41,5691.67");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1323.44,5653.36");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2301.48,5669.46");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3690.66,5660.68");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4230.26,5744.83");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4661.51,5661.11");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5061.45,5629.17");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5481.55,5607.43");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5896.06,5647.36");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=431.20,5439.38");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1518.84,5299.83");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1882.77,5339.62");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3197.11,5512.09");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4012.74,5404.57");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=534.75,5062.35");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1112.43,5257.44");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2354.32,5067.26");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2750.86,5244.72");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3334.45,5084.55");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4536.96,5227.45");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5311.36,5216.73");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=78.69,4812.11");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1375.75,4953.12");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1776.97,4893.39");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3713.45,4870.03");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4295.61,4966.28");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5007.47,4997.83");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1016.45,4762.96");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2326.96,4574.53");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2772.79,4750.20");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3232.53,4567.16");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4592.52,4561.16");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4984.32,4614.18");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5788.26,4728.28");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=63.40,4406.34");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=532.27,4512.50");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1417.04,4520.12");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3849.18,4436.83");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4205.46,4454.52");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5403.71,4447.38");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1098.51,4224.14");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1799.21,4225.58");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2429.01,4182.25");
    }

}

