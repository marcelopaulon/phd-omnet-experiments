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

network DADCA_ACK_n100_u4_a8000m2_6
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
        @display("i=misc/sensor;p=151.36,7982.99");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1434.69,7843.79");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1871.30,7868.58");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2256.07,7853.54");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2791.15,7892.33");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3203.10,7917.22");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=54.52,7583.44");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=543.20,7648.65");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1028.31,7709.59");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2633.67,7552.24");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3682.32,7708.45");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4087.71,7689.40");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5407.86,7731.95");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=679.84,7293.55");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1466.12,7400.72");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1835.78,7401.17");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3186.99,7284.90");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4350.23,7419.07");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5941.76,7504.42");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1123.43,7234.69");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2153.17,7220.64");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3674.50,7066.10");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4790.96,7098.98");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5359.21,7219.22");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=400.84,6889.85");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1966.33,6836.93");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2627.55,6986.43");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3067.85,6923.95");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3401.01,6797.34");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4258.67,7003.74");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5206.73,6840.83");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5964.70,6886.64");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=868.82,6636.10");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1412.47,6597.78");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4221.87,6634.29");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4810.22,6749.21");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5460.09,6580.31");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=408.96,6386.86");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=791.05,6277.01");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2447.92,6346.07");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2842.48,6365.92");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3243.11,6369.90");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1333.79,6140.49");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1716.17,6170.84");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3371.29,6022.40");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4040.65,6110.74");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4741.55,6194.97");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5158.53,6207.57");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5668.20,6122.98");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=214.39,5948.76");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=947.84,5955.72");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1526.82,5825.02");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2015.95,5959.07");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5332.37,5774.15");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1210.97,5644.03");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2515.23,5619.94");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2940.57,5748.76");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3424.86,5620.54");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3780.15,5556.40");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4407.66,5769.63");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5769.18,5723.71");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=20.63,5494.06");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=308.40,5278.59");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=664.99,5417.03");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1742.82,5337.36");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2233.82,5365.70");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4351.99,5351.11");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4798.85,5421.11");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5677.93,5377.90");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1203.95,5058.45");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2812.64,5084.35");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3190.90,5194.09");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4685.63,5078.09");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5151.78,5227.08");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=649.70,4810.89");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1928.62,4937.23");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2443.96,4998.08");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3819.86,4807.38");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5154.61,4846.29");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5526.56,5024.66");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1539.83,4754.45");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2353.89,4636.70");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2794.37,4727.40");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3328.20,4718.09");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4272.87,4572.21");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4843.24,4548.04");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=355.30,4409.64");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=862.48,4454.95");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1403.58,4429.19");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3304.51,4339.00");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3662.31,4425.16");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5764.33,4502.46");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1650.59,4170.52");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2193.22,4201.94");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2652.89,4279.74");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3883.05,4098.54");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4395.87,4132.27");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5292.80,4208.93");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=326.98,3951.08");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1006.58,3873.56");
    }

}

