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

network DADCA_ACK_n20_u4_a8000m2_1
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
        @display("i=misc/sensor;p=1792.37,7783.30");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2303.76,7930.26");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2902.03,7756.43");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4051.14,7875.01");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4812.98,7816.52");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5609.62,7977.44");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=165.81,7564.47");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=540.38,7738.17");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1131.17,7599.38");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2414.84,7530.32");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3477.52,7567.45");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5779.58,7594.30");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=803.98,7414.89");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1809.28,7267.07");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3035.41,7360.42");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5102.89,7457.92");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=357.09,7087.55");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1079.18,7107.59");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1431.80,7050.95");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2277.31,7085.51");
    }

}

