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

network DADCA_ACK_n20_u2_a8000m2_0
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
        @display("i=misc/sensor;p=803.96,7756.83");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1286.46,7949.18");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1854.95,7876.76");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2302.99,7913.70");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4395.02,7849.02");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4796.11,7999.08");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5323.43,7997.98");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1512.74,7573.34");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3761.52,7592.61");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4787.67,7515.50");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5281.66,7521.79");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5598.02,7736.54");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5954.43,7678.55");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2200.74,7439.55");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4221.01,7489.71");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5669.50,7322.00");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=78.58,7136.66");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1242.55,7084.55");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1885.14,7232.49");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2492.64,7183.13");
    }

}

