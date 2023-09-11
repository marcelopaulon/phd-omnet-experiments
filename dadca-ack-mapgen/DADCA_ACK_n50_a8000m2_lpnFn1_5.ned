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

network DADCA_ACK_n50_a8000m2_lpnFn1_5
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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=27.47,166.46");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=29.90,83.17");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=18.00,249.95");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=85.47,207.52");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=120.81,280.39");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=47.37,305.85");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=210.68,368.97");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=287.92,325.99");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=312.30,261.64");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=258.40,217.39");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=210.13,288.51");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=191.66,120.91");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=253.11,111.88");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=69.30,387.49");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=0.39,371.40");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=83.66,463.48");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=1.46,487.74");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=166.18,186.62");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=106.29,340.22");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=195.58,456.63");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=260.56,463.21");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=72.02,532.74");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=83.52,591.72");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=36.31,668.90");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=40.30,740.08");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=98.02,687.62");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=9.94,809.01");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=76.28,815.26");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=142.69,850.86");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=185.02,788.30");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=258.49,779.18");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=235.06,847.29");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=249.71,912.08");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=167.20,944.88");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=237.12,1002.99");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=291.10,956.19");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=351.80,962.48");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=350.88,880.20");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=390.46,834.33");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=419.27,774.12");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=471.02,694.48");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=537.79,660.74");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=551.16,744.13");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=483.91,765.76");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=560.49,820.30");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=480.17,832.87");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=628.85,778.43");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=670.78,848.54");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=757.11,803.30");
    }

}

