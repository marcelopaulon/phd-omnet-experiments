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

network DADCA_ACK_n20_a8000m2_6
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
        @display("i=misc/sensor;p=108.67,7974.25");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=255.95,7966.69");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=396.27,7963.92");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=486.02,7957.91");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=545.55,7972.79");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=782.79,7974.31");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=158.33,7916.89");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=327.48,7937.59");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=677.90,7938.21");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=895.87,7950.37");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=955.40,7919.26");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=565.29,7907.67");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=807.30,7901.87");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=211.40,7865.79");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=289.61,7849.63");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=422.17,7850.20");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=504.42,7868.80");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=626.26,7845.38");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=682.96,7869.91");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=853.87,7860.68");
    }

}

