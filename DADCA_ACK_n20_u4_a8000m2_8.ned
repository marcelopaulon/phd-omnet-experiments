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

network DADCA_ACK_n20_u4_a8000m2_8
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
        @display("i=misc/sensor;p=242.10,7857.92");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1042.59,7779.36");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1512.35,7933.93");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2514.10,7964.24");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2966.05,7972.72");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3333.72,7855.53");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3820.92,7941.13");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4858.69,7935.95");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5551.31,7974.88");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=690.14,7734.30");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1905.25,7728.78");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4152.41,7645.45");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4515.96,7652.34");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5346.18,7602.86");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=190.44,7429.78");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=540.60,7402.50");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1034.66,7415.62");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1481.74,7474.06");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2115.97,7318.21");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2474.86,7500.19");
    }

}

