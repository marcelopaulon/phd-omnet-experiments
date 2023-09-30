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

network DADCA_ACK_n20_u3_a8000m2_4
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
        @display("i=misc/sensor;p=263.74,7925.12");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=639.47,7917.13");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1350.38,7904.99");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3922.31,7795.38");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4773.69,7903.25");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5678.67,7768.78");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=888.96,7581.18");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1688.94,7709.99");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2984.76,7560.61");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3391.57,7646.11");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5123.84,7740.26");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=470.09,7466.55");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1551.78,7287.53");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2014.81,7430.53");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4514.91,7285.53");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4838.12,7486.16");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5832.67,7425.70");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=290.47,7153.32");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=939.68,7204.34");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2392.11,7121.80");
    }

}

