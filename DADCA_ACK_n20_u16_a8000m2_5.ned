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

network DADCA_ACK_n20_u16_a8000m2_5
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
        @display("i=misc/sensor;p=424.86,7828.56");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1896.43,7948.63");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2571.00,7829.16");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3012.41,7896.31");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3552.99,7994.00");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4194.91,7888.59");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5487.81,7977.26");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1444.05,7662.96");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3396.68,7626.35");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3836.97,7579.97");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4724.88,7663.45");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5323.63,7607.31");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=998.71,7467.73");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1777.15,7433.72");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2928.35,7282.35");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4373.64,7474.45");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5695.57,7409.16");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=502.64,7252.13");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1143.50,7145.24");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2010.32,7057.48");
    }

}

