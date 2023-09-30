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

network DADCA_ACK_n100_u16_a8000m2_4
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
        @display("i=misc/sensor;p=1021.12,7765.54");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3122.52,7975.50");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4016.56,7856.35");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4653.92,7889.81");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5238.05,7840.35");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=150.77,7689.08");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=663.53,7752.30");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1947.25,7546.64");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2508.12,7751.85");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3066.41,7505.63");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3422.32,7634.75");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5824.07,7721.12");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=599.75,7289.78");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1125.70,7367.72");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1603.26,7400.37");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3600.59,7327.59");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4072.99,7461.42");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4685.86,7345.89");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5196.50,7425.13");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5872.08,7322.34");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2640.11,7216.62");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3113.07,7087.40");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=80.32,6837.87");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1230.10,6877.58");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1752.39,6945.82");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2155.48,6990.99");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3949.62,6902.59");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4307.52,7008.83");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5036.63,6987.93");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5706.20,6854.32");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=628.22,6728.21");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2528.43,6551.72");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2972.78,6616.67");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3606.89,6666.19");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4709.77,6651.42");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5387.35,6705.43");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=191.54,6379.24");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=807.36,6327.82");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1951.42,6436.17");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4101.83,6433.34");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4775.57,6282.23");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5289.63,6365.70");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5840.78,6313.38");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=606.56,6024.48");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1250.42,6219.23");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2948.53,6138.88");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3666.54,6264.73");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4087.43,6073.67");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=114.45,5789.46");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1836.43,5894.01");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2218.38,5977.87");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3268.14,5994.54");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3738.49,5909.60");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4764.02,5863.39");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5138.18,5878.13");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=561.68,5540.19");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1134.07,5656.21");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1509.73,5633.40");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2847.16,5716.50");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4317.36,5719.09");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4873.67,5528.10");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1052.57,5283.91");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2034.38,5326.62");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2325.88,5524.77");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3314.95,5364.43");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3953.97,5450.59");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4454.88,5313.42");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5467.23,5457.74");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=189.22,5173.28");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=653.74,5078.75");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2911.74,5119.38");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3640.19,5227.96");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5802.10,5246.41");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1534.19,4964.25");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1948.62,4894.02");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2354.82,4786.49");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3356.72,4869.50");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3817.21,4869.93");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4210.07,4901.15");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4719.58,4849.87");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5236.46,5030.12");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=326.19,4613.22");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=978.59,4658.67");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3133.51,4599.03");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4475.14,4538.22");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5043.52,4539.82");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5401.95,4554.79");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5951.39,4683.82");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1533.88,4328.61");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1955.74,4322.17");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2623.53,4343.75");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3862.39,4338.75");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5940.00,4316.84");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=553.69,4141.47");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1146.58,4074.30");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2962.02,4058.55");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3401.59,4199.81");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4294.19,4192.32");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5214.11,4138.72");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=106.75,3955.86");
    }

}

