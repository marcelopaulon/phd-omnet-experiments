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

network DADCA_ACK_n20_a8000m2_3
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
        @display("i=misc/sensor;p=370.97,7824.23");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1324.31,7943.01");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1837.25,7864.59");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2346.29,7818.29");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3976.78,7842.41");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4329.04,7901.43");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5007.75,7918.24");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5562.88,7977.65");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2700.61,7577.00");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3525.34,7653.61");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4415.22,7557.42");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5488.02,7632.90");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5989.13,7587.73");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=149.35,7464.36");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=584.71,7269.70");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=928.76,7338.46");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1525.86,7409.29");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1884.33,7481.99");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3028.13,7288.59");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4116.33,7309.92");
    }

}

