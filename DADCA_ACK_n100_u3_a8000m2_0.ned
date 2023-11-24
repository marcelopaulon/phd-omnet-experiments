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

network DADCA_ACK_n100_u3_a8000m2_0
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
        @display("i=misc/sensor;p=335.36,7990.09");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=728.57,7938.93");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1774.06,7870.18");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2298.98,7907.18");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3821.66,7817.82");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4188.60,7903.25");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5200.35,7765.14");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5826.30,7864.51");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=398.45,7523.58");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=929.22,7536.32");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2605.61,7542.70");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2922.53,7718.75");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4498.11,7728.53");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1622.80,7264.24");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2038.34,7334.95");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3431.47,7418.39");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4104.73,7333.53");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4648.61,7392.92");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5125.58,7274.18");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5749.05,7468.86");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=187.19,7173.08");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1063.24,7205.36");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2416.87,7102.07");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2956.20,7250.83");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3845.48,7073.82");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=785.04,6964.69");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1203.63,6806.15");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5036.88,6880.08");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5432.20,6931.93");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5921.01,6954.75");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=392.54,6558.59");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=790.20,6542.00");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1620.30,6736.76");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2072.29,6731.60");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2699.49,6676.23");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3241.73,6743.23");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3758.82,6594.56");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4276.95,6554.97");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4624.35,6736.94");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4976.52,6530.62");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5372.41,6540.37");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5726.91,6523.49");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1990.31,6332.11");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2427.31,6320.44");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3066.13,6388.17");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3445.17,6385.53");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=118.81,6164.71");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1007.80,6250.37");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1599.85,6262.71");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4517.09,6147.25");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5868.38,6095.71");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=544.91,5801.71");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2024.69,5969.19");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3005.54,5932.44");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3905.82,6002.34");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4256.76,5898.51");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4692.18,5804.36");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=170.29,5763.55");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1199.85,5765.83");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2509.03,5732.12");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4079.14,5553.66");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5289.77,5727.92");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1063.84,5385.47");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1569.62,5397.25");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2074.75,5349.60");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3143.41,5344.22");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3709.53,5424.76");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4408.92,5359.58");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4903.62,5486.94");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5680.47,5424.91");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=500.79,5223.43");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1354.38,5101.34");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2702.30,5172.18");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5250.00,5128.59");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=895.00,4823.13");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1664.64,4804.39");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2029.38,4924.05");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2670.71,4796.55");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2994.20,4939.94");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3521.69,4979.23");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4211.56,4888.73");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4901.29,4947.91");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5432.31,4815.25");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5964.86,4848.94");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3893.41,4542.94");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5723.55,4554.24");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=173.86,4456.19");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=642.50,4386.44");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1296.00,4408.32");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2051.48,4400.02");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3146.31,4534.38");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4512.64,4410.72");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4941.99,4420.32");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1697.56,4065.08");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2387.30,4160.61");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2742.41,4235.04");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3627.42,4265.14");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4149.42,4236.67");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5433.90,4257.32");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=377.56,3907.23");
    }

}

