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

network DADCA_ACK_n100_u16_a8000m2_6
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
        @display("i=misc/sensor;p=657.60,7957.99");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1343.16,7935.42");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2308.38,7872.79");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2775.43,7782.48");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3607.42,7956.58");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4369.93,7879.52");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=259.77,7581.78");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1097.10,7603.92");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1827.31,7647.58");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3313.62,7727.60");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3956.63,7581.17");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5144.79,7703.98");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5764.57,7661.00");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1612.62,7365.01");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2470.77,7399.85");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2919.99,7423.18");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4335.30,7352.38");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4933.08,7329.74");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=941.40,7097.59");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1303.38,7141.91");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1934.52,7110.41");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2875.70,7021.93");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3317.85,7081.26");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3675.98,7188.80");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4653.51,7103.12");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5340.71,7050.35");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5880.18,7078.64");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=220.38,6990.58");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2466.87,6944.47");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4066.25,6940.33");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5026.42,6784.15");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=345.04,6549.83");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=869.92,6748.31");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1336.54,6712.94");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1822.47,6750.89");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3051.37,6618.25");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3715.46,6584.06");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4349.19,6521.26");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1234.98,6357.21");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1896.24,6302.45");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2628.10,6493.98");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4706.20,6493.88");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5201.72,6368.57");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5771.73,6460.65");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=350.97,6036.94");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=851.00,6083.56");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1624.94,6072.00");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2926.82,6186.23");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3385.33,6083.10");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4036.32,6258.48");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4526.08,6096.72");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1870.29,5819.29");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2386.68,5943.88");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2741.91,5786.07");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4266.34,5861.29");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5135.03,5786.31");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5580.88,5943.11");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1230.71,5556.57");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3270.61,5709.07");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3967.91,5672.75");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4790.97,5702.52");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=311.30,5462.12");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=694.89,5514.15");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1605.75,5406.79");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2265.01,5378.85");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2618.49,5285.25");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4527.94,5465.66");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5027.10,5293.51");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5397.43,5510.52");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5758.12,5368.53");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=438.32,5119.23");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1115.99,5060.95");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3024.29,5155.87");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3501.97,5169.14");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4072.59,5046.51");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4539.04,5060.39");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5468.56,5158.59");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=94.67,4930.57");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=625.39,4821.09");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2436.81,4923.08");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=8.63,4565.00");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1602.96,4629.28");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2764.25,4655.53");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3384.22,4755.32");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3894.88,4593.12");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4474.60,4662.20");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5090.98,4556.03");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5430.85,4642.08");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5758.37,4771.33");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=335.89,4343.60");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=971.00,4431.60");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1337.67,4342.20");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2244.22,4532.09");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1640.73,4055.55");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2013.68,4178.32");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2515.74,4051.84");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3277.00,4161.90");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3628.85,4047.54");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4447.08,4132.33");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5545.92,4154.20");
    }

}

