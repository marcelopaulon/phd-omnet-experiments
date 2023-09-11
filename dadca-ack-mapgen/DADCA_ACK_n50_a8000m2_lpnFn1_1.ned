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

network DADCA_ACK_n50_a8000m2_lpnFn1_1
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
        @display("i=misc/sensor,p=194.02,119.08");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=150.27,59.31");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=217.64,59.53");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=277.15,77.67");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=288.63,145.26");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=350.50,147.75");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=372.50,57.46");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=402.61,177.92");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=445.49,240.29");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=387.84,293.76");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=444.96,329.20");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=347.87,208.63");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=529.26,247.52");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=558.86,340.99");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=482.44,378.37");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=556.43,434.89");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=644.00,395.38");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=677.88,447.84");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=723.44,402.31");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=660.46,287.37");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=383.05,361.59");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=531.41,113.77");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=464.32,123.27");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=7.78,91.82");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=41.21,169.68");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=173.24,194.66");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=238.49,220.38");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=159.20,268.99");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=237.00,282.67");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=304.34,277.36");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=389.96,432.11");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=449.25,469.08");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=419.27,536.67");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=503.33,562.53");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=521.29,621.72");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=464.83,686.98");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=614.35,703.80");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=625.58,775.23");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=694.04,798.81");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=761.69,739.91");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=810.27,780.45");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=774.01,836.29");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=890.32,733.18");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=864.46,673.08");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=792.26,676.42");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=829.19,594.51");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=754.50,596.26");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=683.88,613.19");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=971.00,696.89");
    }

}

