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

network DADCA_ACK_n50_u8_a8000m2_2
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
        @display("i=misc/sensor;p=1156.40,7852.72");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2555.80,7921.85");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4293.10,7807.46");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4908.00,7982.60");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5600.08,7941.87");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=241.92,7661.87");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=710.57,7505.09");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1366.58,7516.74");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1751.81,7541.84");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2365.88,7597.32");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2966.43,7673.84");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3595.59,7712.25");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4677.97,7543.22");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3170.39,7364.23");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3572.56,7274.32");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3942.47,7422.91");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4266.74,7284.36");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5250.87,7406.61");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5720.79,7297.93");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=238.59,7057.02");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=668.84,7118.40");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1227.61,7190.03");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2508.72,7248.10");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4591.80,7056.49");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2184.73,7007.84");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2894.33,6833.47");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3568.93,6763.38");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5378.44,6954.68");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5863.07,6890.46");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=140.66,6687.74");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=625.83,6697.35");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=995.72,6717.72");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1337.30,6556.61");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1844.34,6740.21");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4123.17,6754.53");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4686.92,6548.24");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5189.40,6652.44");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2259.95,6465.65");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2684.06,6472.20");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3277.05,6416.41");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3863.32,6455.50");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4301.38,6315.68");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5593.88,6351.64");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5960.77,6354.09");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=495.86,6250.92");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1600.52,6256.68");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3153.15,6054.62");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4970.52,6167.88");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=329.69,5791.80");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=978.18,6000.71");
    }

}

