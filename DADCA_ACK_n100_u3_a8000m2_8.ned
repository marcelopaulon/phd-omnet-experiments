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

network DADCA_ACK_n100_u3_a8000m2_8
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
        @display("i=misc/sensor;p=516.52,7824.68");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1496.69,7870.29");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3958.24,7810.71");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4356.48,7890.85");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5313.00,7825.18");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5829.27,7861.14");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2023.31,7593.07");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3566.63,7710.88");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4629.82,7567.62");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4988.48,7663.98");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=545.06,7473.75");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=935.79,7499.76");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2331.52,7258.04");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4002.96,7383.57");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5501.58,7451.01");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5854.23,7420.47");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=99.13,7244.06");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1439.81,7090.09");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2768.61,7085.88");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3456.53,7213.67");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3808.55,7044.90");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4697.42,7010.50");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5228.86,7145.68");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=320.62,6960.52");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=891.17,6987.22");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2474.70,6787.76");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=11.66,6585.30");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1203.41,6603.04");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1706.64,6638.97");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2978.59,6702.72");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3362.91,6749.83");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4046.68,6658.34");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5504.16,6631.33");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=476.86,6450.27");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=862.75,6359.22");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2244.64,6432.01");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4479.68,6471.47");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=127.06,6110.49");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=524.34,6042.68");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1876.86,6188.24");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2542.15,6130.74");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2937.51,6203.66");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3280.46,6047.79");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3615.02,6179.37");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4131.14,6140.42");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4989.23,6163.34");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5570.52,6084.49");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=886.43,5998.70");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1527.15,6006.06");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=421.93,5662.99");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=826.90,5634.78");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1439.02,5638.61");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2071.38,5685.66");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2689.09,5678.77");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3237.31,5664.98");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3827.48,5586.18");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4534.52,5737.44");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5012.27,5528.84");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5426.96,5620.01");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5824.49,5725.07");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1232.90,5315.02");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1584.23,5308.15");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2123.86,5299.19");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4275.32,5466.73");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5794.62,5348.46");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=726.48,5080.97");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2487.93,5269.84");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2819.07,5045.26");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3331.93,5059.36");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3666.25,5177.34");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4631.24,5127.70");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5261.96,5266.72");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1299.64,4855.50");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1901.80,5009.27");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2287.02,4925.60");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4126.90,4782.89");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5236.14,4898.98");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5654.27,5019.25");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5987.34,4837.40");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=780.82,4718.94");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3153.37,4561.89");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3596.17,4692.23");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4462.21,4574.83");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4795.47,4693.65");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5368.07,4565.77");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1043.08,4470.45");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1940.33,4445.84");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2338.60,4298.23");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4027.30,4288.06");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5784.74,4316.17");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=309.47,4234.43");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=839.97,4143.63");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1640.82,4111.85");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2698.57,4285.63");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3656.79,4282.52");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5156.25,4221.02");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=374.97,3855.03");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1282.19,3847.70");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1982.77,3861.53");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2357.24,3944.25");
    }

}

