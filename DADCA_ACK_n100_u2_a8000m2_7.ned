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

network DADCA_ACK_n100_u2_a8000m2_7
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
        @display("i=misc/sensor;p=222.97,7986.09");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=903.55,7867.42");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2590.52,7850.73");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3191.03,7871.67");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=410.88,7509.14");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1582.56,7582.00");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1987.06,7696.45");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3817.41,7658.39");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4506.64,7664.85");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4937.00,7605.05");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5420.28,7744.39");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=750.26,7405.56");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2220.23,7351.36");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2854.07,7318.97");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3389.47,7492.08");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3749.00,7297.89");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1124.86,7159.40");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1700.29,7157.74");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2463.03,7082.31");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3393.06,7065.29");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4952.28,7242.78");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5272.95,7088.28");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5704.48,7124.31");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=329.11,6991.13");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=821.64,6957.52");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1457.06,6767.94");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2265.49,6772.96");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4231.29,6995.20");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1106.51,6527.38");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1928.03,6650.62");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2917.00,6646.82");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3745.40,6707.09");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4873.71,6523.47");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=651.54,6499.34");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2542.89,6282.92");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3357.88,6506.63");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3993.00,6347.93");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5333.71,6400.57");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=304.89,6138.88");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1174.98,6041.94");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1640.77,6200.06");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2128.58,6172.75");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3235.19,6060.61");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3657.38,6035.64");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4668.48,6211.22");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5979.29,6100.45");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=754.60,5930.05");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2798.10,5961.87");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4297.77,5775.29");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4918.12,5942.00");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5601.37,5956.91");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=307.58,5725.38");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=902.30,5595.12");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1335.19,5556.21");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1736.79,5636.14");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2477.90,5579.88");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3585.46,5595.60");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5874.67,5632.03");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=432.06,5331.27");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3058.36,5510.38");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3977.39,5498.76");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4604.71,5414.67");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5099.43,5495.03");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1213.86,5188.89");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2669.60,5115.44");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5122.03,5063.53");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5549.09,5055.56");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=207.57,5015.81");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=816.94,4824.47");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1654.70,4919.95");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2017.44,5020.66");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3086.39,4943.47");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3681.48,4892.94");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4433.30,4976.86");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5943.78,4942.36");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=125.26,4577.08");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1379.04,4638.92");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2288.05,4692.97");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2866.15,4593.91");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5631.41,4595.59");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=482.30,4500.77");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1746.05,4434.49");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3363.51,4384.48");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4006.52,4514.77");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4810.07,4460.63");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=636.92,4131.75");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1043.75,4182.98");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2058.84,4146.70");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2559.97,4098.92");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4398.68,4178.02");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5281.80,4129.09");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=50.40,4034.92");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=479.46,3811.48");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1370.00,3960.88");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1860.45,3852.38");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3012.20,3857.68");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3732.67,3969.09");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4109.19,3933.34");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4570.59,3839.85");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5875.22,4010.73");
    }

}

