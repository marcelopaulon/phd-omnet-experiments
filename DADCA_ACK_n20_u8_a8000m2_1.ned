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

network DADCA_ACK_n20_u8_a8000m2_1
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
        @display("i=misc/sensor;p=1657.60,7779.64");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2228.34,7994.76");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3193.99,7772.93");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3775.56,7787.07");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=992.62,7605.58");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2123.44,7543.90");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2567.57,7718.81");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5517.39,7568.45");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5993.96,7732.07");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=264.31,7345.13");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1596.47,7421.55");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3527.68,7287.05");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5157.07,7355.35");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=519.10,7043.96");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1304.15,7207.99");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1614.92,7010.30");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1962.74,7116.82");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2830.50,7241.67");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4074.42,7081.43");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5667.30,7245.68");
    }

}

