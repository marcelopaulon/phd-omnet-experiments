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

network DADCA_ACK_n20_u4_a8000m2_7
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
        @display("i=misc/sensor;p=975.75,7799.80");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1886.13,7904.41");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2381.29,7991.41");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2710.15,7865.98");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3232.50,7973.75");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3924.16,7914.61");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5608.17,7996.05");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=417.27,7670.91");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3137.06,7635.22");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4206.26,7678.94");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5060.24,7628.56");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5810.17,7569.20");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=181.17,7302.33");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1418.90,7379.19");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2052.39,7464.02");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2527.39,7465.69");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2873.84,7388.92");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3682.36,7481.33");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4718.41,7425.26");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=977.83,7177.51");
    }

}

