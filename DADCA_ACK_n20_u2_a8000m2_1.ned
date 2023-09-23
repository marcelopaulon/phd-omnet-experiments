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

network DADCA_ACK_n20_u2_a8000m2_1
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
        @display("i=misc/sensor;p=984.74,7832.79");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1326.04,7926.37");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2147.68,7848.26");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3100.02,7869.17");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4229.67,7873.95");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5008.26,7916.78");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=274.50,7619.30");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2770.69,7530.21");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3820.57,7664.20");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5224.64,7525.87");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5785.88,7594.60");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=844.94,7435.74");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1294.82,7428.89");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1795.09,7293.20");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2401.12,7316.15");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3262.78,7456.45");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3924.92,7264.01");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4501.66,7438.31");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=973.33,7102.85");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4816.85,7181.87");
    }

}

