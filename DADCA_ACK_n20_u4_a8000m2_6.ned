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

network DADCA_ACK_n20_u4_a8000m2_6
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
        @display("i=misc/sensor;p=8.72,7922.13");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=607.19,7803.12");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1033.72,7897.63");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2246.65,7851.29");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4282.88,7775.02");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4699.78,7913.48");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=827.37,7517.53");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1805.18,7628.92");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5235.76,7681.84");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5823.23,7744.13");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=134.95,7325.97");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1277.35,7374.45");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4415.84,7357.47");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5624.97,7308.56");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=715.26,7138.27");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1806.00,7082.65");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2366.47,7149.39");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2809.60,7189.40");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3928.14,7216.08");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4822.84,7234.57");
    }

}

