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

network DADCA_ACK_n20_u16_a8000m2_9
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
        @display("i=misc/sensor;p=1220.59,7828.63");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1880.00,7799.32");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4244.05,7968.77");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=303.99,7539.88");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=710.67,7667.03");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2202.29,7644.19");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2822.37,7691.50");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3728.04,7655.38");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4677.88,7519.13");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5285.12,7539.63");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5862.71,7615.53");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1352.66,7345.94");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2007.67,7287.90");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2531.06,7417.59");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=35.15,7070.55");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=488.97,7101.46");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=813.47,7245.31");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1664.21,7184.30");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3040.06,7249.48");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3597.78,7043.08");
    }

}

