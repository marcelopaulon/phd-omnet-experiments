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

network DADCA_ACK_n50_u3_a8000m2_2
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
        @display("i=misc/sensor;p=1446.14,7864.38");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2071.29,7765.94");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2800.14,7998.11");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3243.99,7943.86");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3699.98,7957.95");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4476.76,7785.61");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5345.44,7856.40");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5724.39,7829.08");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=729.76,7601.61");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2605.52,7548.32");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3489.37,7529.62");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4022.48,7725.62");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4996.43,7528.95");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=198.34,7341.02");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1124.35,7275.35");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1579.07,7386.63");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3117.53,7490.12");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4153.40,7320.92");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5378.56,7330.74");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5887.18,7500.26");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=533.51,7146.39");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2159.54,7055.59");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2545.13,7116.65");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3117.83,7109.06");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3532.03,7030.47");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4957.19,7116.19");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5741.78,7075.82");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=825.22,6829.09");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1473.37,6897.08");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2801.23,6828.64");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4549.89,6860.71");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1933.47,6611.64");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2499.90,6633.80");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3482.61,6640.67");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4045.13,6678.91");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4827.23,6575.83");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5181.12,6671.61");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=529.22,6365.09");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1474.45,6341.81");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2831.41,6426.71");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3834.35,6381.62");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4498.57,6303.18");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5670.40,6284.84");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=359.17,6057.94");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2180.30,6205.50");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2757.04,6047.95");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3314.43,6094.09");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4128.08,6174.18");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5050.38,6073.74");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1039.72,5913.78");
    }

}

