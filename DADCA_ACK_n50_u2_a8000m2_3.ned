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

network DADCA_ACK_n50_u2_a8000m2_3
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
        @display("i=misc/sensor;p=748.56,7764.12");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1457.48,7982.61");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2148.12,7921.42");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3105.43,7876.54");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3771.41,7808.69");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4354.54,7926.01");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1922.75,7570.90");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2574.89,7516.64");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3127.10,7517.57");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4672.63,7730.20");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5155.03,7677.26");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5704.25,7562.13");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=299.38,7482.17");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=592.52,7263.40");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1257.35,7379.04");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3949.33,7500.86");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4898.87,7338.82");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=960.74,7181.05");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1596.23,7093.51");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2402.88,7163.25");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2907.84,7137.07");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3706.54,7203.72");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4478.71,7178.37");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5244.28,7250.94");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=148.87,6890.63");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=518.03,6908.94");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3465.00,6892.99");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4134.55,6937.74");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5881.20,6923.08");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1044.64,6745.86");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3697.25,6526.96");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4471.61,6562.01");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4913.67,6678.10");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=131.29,6494.95");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=609.10,6472.10");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1495.70,6367.99");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1823.07,6502.69");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2344.79,6475.47");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2886.06,6441.43");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4976.70,6311.53");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5486.86,6467.93");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=721.92,6131.40");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1164.22,6094.62");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1850.81,6049.03");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2342.47,6068.95");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3293.42,6140.95");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3888.85,6047.64");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4619.15,6178.11");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5567.61,6100.84");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=189.04,5862.29");
    }

}

