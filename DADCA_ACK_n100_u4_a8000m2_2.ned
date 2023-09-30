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

network DADCA_ACK_n100_u4_a8000m2_2
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
        @display("i=misc/sensor;p=831.46,7865.28");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1242.40,7955.75");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1671.61,7787.44");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2090.35,7943.34");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3677.79,7792.70");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4835.27,7859.31");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5557.64,7990.63");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2426.87,7554.49");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2759.19,7665.61");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3112.31,7599.72");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4188.19,7562.74");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5343.88,7563.87");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=360.42,7447.99");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1468.46,7373.75");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3741.97,7267.96");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4852.71,7469.48");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5950.18,7355.84");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=816.66,7219.39");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1997.29,7257.24");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2356.60,7165.73");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2790.61,7134.01");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3378.63,7076.17");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4427.03,7199.48");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5053.03,7038.70");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5757.94,7018.38");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=15.07,6910.65");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=753.80,6766.43");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1191.45,6936.44");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1770.79,6990.30");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2767.22,6776.52");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3838.72,6906.83");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4339.78,6831.70");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4765.59,6769.62");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=380.10,6755.73");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2142.38,6719.35");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3121.98,6748.29");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4039.05,6551.75");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5273.91,6735.71");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=343.26,6405.14");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=695.11,6336.49");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1028.24,6511.24");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2174.41,6370.22");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2734.96,6397.34");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3413.20,6410.82");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5909.99,6467.51");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1815.57,6173.61");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3224.87,6101.66");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4667.50,6263.20");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5254.42,6125.07");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5676.27,6191.85");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=45.67,5797.28");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=406.51,5808.47");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1310.07,5960.32");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2651.64,5966.13");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3615.00,6006.95");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4209.01,5847.16");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4843.38,5933.43");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5502.67,5877.18");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1631.08,5532.12");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1967.32,5743.51");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2384.60,5638.27");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3029.12,5588.77");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4678.59,5585.16");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5065.34,5566.31");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=80.49,5419.75");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1192.90,5511.29");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2751.90,5351.24");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3389.84,5515.44");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4252.17,5407.77");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5501.26,5504.90");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=170.28,5060.59");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=597.85,5276.77");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1901.76,5236.85");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3424.04,5059.30");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3941.51,5157.45");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5171.59,5118.58");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5736.44,5140.71");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=692.26,4919.94");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2369.68,4974.05");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3034.57,4922.69");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4362.75,4846.74");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4789.73,5020.63");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1668.80,4746.44");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2514.95,4619.07");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3179.19,4589.64");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3651.26,4543.54");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4012.01,4725.77");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4958.13,4703.90");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5354.71,4670.50");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5983.14,4603.14");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=363.49,4327.09");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=921.20,4457.93");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1451.53,4389.07");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2025.70,4492.44");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4476.53,4463.85");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1123.21,4089.29");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2509.38,4050.55");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2833.32,4213.86");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3543.06,4093.16");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4142.24,4273.53");
    }

}

